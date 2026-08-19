// rsf_node2 - ROS 2 driver node for the Hokuyo RSF-X001 localization sensor.
//
// SPDX-License-Identifier: Apache-2.0
//
// The node is deliberately thin. All protocol work lives in rsf_library and all
// message building in rsf_ros/conversions.hpp, so what remains here is the part
// that genuinely differs between ROS distributions: declaring parameters,
// creating publishers, and forwarding. This is the only file in the package
// that includes rclcpp.
//
// Threading: rsf::Client runs its receive loop on its own thread and the
// callbacks below are invoked from it. Those callbacks build the ROS message
// and hand it to a PublishLane; the actual publish() happens on that lane's
// own thread, one lane per topic.
//
// That separation is not decoration. A rclcpp publisher can block - a RELIABLE
// writer whose history is full waits for its subscribers - and the node emits
// thousands of messages a second. With publish() called inline, one slow
// subscriber stopped every topic at once and stopped the thread draining the
// TCP socket with them, so the sensor link backed up and data arrived in
// multi-second bursts. Per-topic lanes confine a blocked writer to its own
// topic: it loses queued samples there and nothing else is affected.

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <nmea_msgs/msg/gprmc.hpp>
#include <nmea_msgs/msg/gpzda.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rsf/rsf.hpp"
#include "rsf_ros/conversions.hpp"
#include "rsf_ros/driver_config.hpp"

namespace rsf_node2 {

// One publishing lane: a bounded queue and the thread that drains it.
//
// There is a lane per topic rather than one shared queue, because publish() can
// block for a long time - a RELIABLE writer whose history is full waits for its
// subscribers - and with a single queue the topic behind a slow subscriber
// stalls every other topic behind it. A lane confines that to the topic that
// actually has the slow consumer.
//
// The queue discards the oldest entry when full, so a slow subscriber costs
// samples on its own topic instead of applying back pressure to the sensor
// link. Discards and the worst observed publish() duration are recorded, so the
// offending topic can be named instead of guessed at.
class PublishLane {
 public:
  PublishLane(std::string name, std::size_t capacity)
      : name_(std::move(name)),
        capacity_(capacity > 0 ? capacity : 1),
        thread_(&PublishLane::run, this) {}

  ~PublishLane() { stop(); }

  PublishLane(const PublishLane&) = delete;
  PublishLane& operator=(const PublishLane&) = delete;

  // Never blocks. `work` may hold move-only state such as a unique_ptr message.
  template <typename Work>
  void post(Work&& work) {
    auto job = std::unique_ptr<JobBase>(new Job<std::decay_t<Work>>(std::forward<Work>(work)));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) {
        return;
      }
      if (jobs_.size() >= capacity_) {
        jobs_.pop_front();
        ++dropped_;
      }
      jobs_.push_back(std::move(job));
    }
    condition_.notify_one();
  }

  // Stops accepting work and lets the thread finish what is already queued.
  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) {
        return;
      }
      running_ = false;
    }
    condition_.notify_all();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  struct Report {
    std::string name;
    std::uint64_t dropped_since = 0;
    std::size_t depth = 0;
    double worst_publish_seconds = 0.0;
  };

  // Returns activity since the previous call and clears the interval counters.
  Report takeReport() {
    std::lock_guard<std::mutex> lock(mutex_);
    Report report;
    report.name = name_;
    report.dropped_since = dropped_ - reported_;
    report.depth = jobs_.size();
    report.worst_publish_seconds = worst_publish_seconds_;
    reported_ = dropped_;
    worst_publish_seconds_ = 0.0;
    return report;
  }

 private:
  struct JobBase {
    virtual ~JobBase() = default;
    virtual void run() = 0;
  };

  template <typename Work>
  struct Job : JobBase {
    explicit Job(Work&& work) : work(std::move(work)) {}
    void run() override { work(); }
    Work work;
  };

  void run() {
    for (;;) {
      std::unique_ptr<JobBase> job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return !jobs_.empty() || !running_; });
        if (jobs_.empty()) {
          return;  // stopped and drained
        }
        job = std::move(jobs_.front());
        jobs_.pop_front();
      }

      // Outside the lock: this is the call that may block, and timing it is
      // what turns "something is slow" into a named topic.
      const auto started = std::chrono::steady_clock::now();
      job->run();
      const double seconds =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      if (seconds > 0.001) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (seconds > worst_publish_seconds_) {
          worst_publish_seconds_ = seconds;
        }
      }
    }
  }

  std::string name_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<std::unique_ptr<JobBase>> jobs_;
  std::size_t capacity_;
  std::uint64_t dropped_ = 0;
  std::uint64_t reported_ = 0;
  double worst_publish_seconds_ = 0.0;
  bool running_ = true;
  std::thread thread_;
};

class RsfNode : public rclcpp::Node {
 public:
  RsfNode() : rclcpp::Node("rsf_node") {
    config_ = readParameters();

    const std::string problem = config_.validate();
    if (!problem.empty()) {
      RCLCPP_FATAL(get_logger(), "invalid configuration: %s", problem.c_str());
      throw std::runtime_error(problem);
    }

    createPublishers();
    createSubscriptions();

    if (config_.broadcast_tf) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    createPublishLanes();

    // Reports discarded samples per lane, so a subscriber that cannot keep up
    // shows up in the log by name instead of as unexplained gaps.
    report_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { reportDrops(); });

    RCLCPP_INFO(get_logger(), "connecting to RSF-X001 at %s:%d", config_.ip_address.c_str(),
                config_.port);

    client_ = std::make_unique<rsf::Client>(config_.toClientConfig(), makeCallbacks());
    client_->start();
  }

  ~RsfNode() override {
    // Stop receiving first, so nothing new is queued, then let the publishing
    // thread drain into publishers that are still alive.
    if (client_) {
      client_->stop();
      client_->disconnect();
    }
    for (const auto& lane : lanes_) {
      lane->stop();
    }
  }

 private:
  // ******************** Setup ********************

  // Declares every parameter with the struct default, then reads it back, so
  // that the defaults are stated once in rsf_ros/driver_config.hpp.
  rsf_ros::DriverConfig readParameters() {
    rsf_ros::DriverConfig config;

    const auto get = [this](const std::string& name, auto& value) {
      value = this->declare_parameter(name, value);
    };

    get("ip_address", config.ip_address);
    get("port", config.port);
    get("start_streaming_on_connect", config.start_streaming_on_connect);
    get("start_rsf_on_connect", config.start_rsf_on_connect);
    get("wire_layout", config.wire_layout);
    get("broadcast_tf", config.broadcast_tf);
    get("tf_decimation", config.tf_decimation);
    get("publish_lidar_rate_odom", config.publish_lidar_rate_odom);
    get("queue_size", config.queue_size);
    get("publish_queue_size", config.publish_queue_size);
    get("publish_text_on_change_only", config.publish_text_on_change_only);
    get("text_keepalive_hz", config.text_keepalive_hz);
    get("odom_decimation", config.odom_decimation);
    get("diagnostic_status_name", config.diagnostic_status_name);
    get("enable_set_ip_address", config.enable_set_ip_address);

    get("fix_topic", config.topics.fix);
    get("gga_topic", config.topics.gga);
    get("rmc_topic", config.topics.rmc);
    get("zda_topic", config.topics.zda);
    get("point_cloud_topic", config.topics.point_cloud);
    get("imu_topic", config.topics.imu);
    get("lio_odom_topic", config.topics.lio_odom);
    get("switch_fix_topic", config.topics.switch_fix);
    get("utm_odom_topic", config.topics.utm_odom);
    get("switch_odom_topic", config.topics.switch_odom);
    get("switch_odom_state_topic", config.topics.switch_odom_state);
    get("switch_odom_type_topic", config.topics.switch_odom_type);
    get("switch_fix_state_topic", config.topics.switch_fix_state);
    get("switch_fix_type_topic", config.topics.switch_fix_type);
    get("diagnostics_topic", config.topics.diagnostics);
    get("lidar_rate_odom_topic", config.topics.lidar_rate_odom);
    get("command_topic", config.topics.command);
    get("set_ip_address_topic", config.topics.set_ip_address);

    get("odom_frame", config.frames.odom);
    get("utm_frame", config.frames.utm);
    get("lidar_frame", config.frames.lidar);
    get("imu_frame", config.frames.imu);
    get("gnss_frame", config.frames.gnss);

    return config;
  }

  void createPublishers() {
    const rclcpp::QoS qos(rclcpp::KeepLast(static_cast<std::size_t>(config_.queue_size)));
    const rsf_ros::TopicConfig& topics = config_.topics;

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(topics.fix, qos);
    gga_pub_ = create_publisher<nmea_msgs::msg::Gpgga>(topics.gga, qos);
    rmc_pub_ = create_publisher<nmea_msgs::msg::Gprmc>(topics.rmc, qos);
    zda_pub_ = create_publisher<nmea_msgs::msg::Gpzda>(topics.zda, qos);
    point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topics.point_cloud, qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(topics.imu, qos);
    lio_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(topics.lio_odom, qos);
    switch_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(topics.switch_fix, qos);
    utm_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(topics.utm_odom, qos);
    switch_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(topics.switch_odom, qos);
    switch_odom_state_pub_ = create_publisher<std_msgs::msg::String>(topics.switch_odom_state, qos);
    switch_odom_type_pub_ = create_publisher<std_msgs::msg::String>(topics.switch_odom_type, qos);
    switch_fix_state_pub_ = create_publisher<std_msgs::msg::String>(topics.switch_fix_state, qos);
    switch_fix_type_pub_ = create_publisher<std_msgs::msg::String>(topics.switch_fix_type, qos);
    diagnostics_pub_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(topics.diagnostics, qos);

    if (config_.publish_lidar_rate_odom) {
      lidar_rate_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(topics.lidar_rate_odom, qos);
    }
  }

  // One lane per topic, so that a topic whose subscriber has fallen behind
  // cannot hold up any other topic. The 1 kHz streams get their own lanes
  // because they are the ones a slow consumer stalls first; the low rate topics
  // share lanes, since they cannot fill one between them.
  void createPublishLanes() {
    const auto capacity = static_cast<std::size_t>(config_.publish_queue_size);
    const auto lane = [this, capacity](const char* name) {
      lanes_.push_back(std::make_unique<PublishLane>(name, capacity));
      return lanes_.back().get();
    };

    cloud_lane_ = lane("point_cloud");
    imu_lane_ = lane("imu");
    lio_odom_lane_ = lane("lio_odom");
    switch_odom_lane_ = lane("switch_odom");
    utm_odom_lane_ = lane("utm_odom");
    fix_lane_ = lane("nav_sat_fix");
    tf_lane_ = lane("tf");
    lidar_rate_lane_ = lane("lidar_rate_odom");
    slow_lane_ = lane("gnss_text_diagnostics");
  }

  void createSubscriptions() {
    // Commands are latched so that one sent before the node is up still arrives.
    rclcpp::QoS command_qos(rclcpp::KeepLast(10));
    command_qos.reliable();
    command_qos.transient_local();

    command_sub_ = create_subscription<std_msgs::msg::UInt8>(
        config_.topics.command, command_qos,
        [this](std_msgs::msg::UInt8::SharedPtr msg) { onCommand(msg->data); });

    if (config_.enable_set_ip_address) {
      set_ip_address_sub_ = create_subscription<std_msgs::msg::String>(
          config_.topics.set_ip_address, command_qos,
          [this](std_msgs::msg::String::SharedPtr msg) { onSetIpAddress(msg->data); });
    }
  }

  rsf::ClientCallbacks makeCallbacks() {
    rsf::ClientCallbacks callbacks;
    callbacks.on_odometry = [this](const rsf::Odometry& v) { onOdometry(v); };
    callbacks.on_point_cloud = [this](const rsf::PointCloud& v) { onPointCloud(v); };
    callbacks.on_imu = [this](const rsf::Imu& v) { onImu(v); };
    callbacks.on_nav_sat_fix = [this](const rsf::NavSatFix& v) { onNavSatFix(v); };
    callbacks.on_gga = [this](const rsf::GgaSentence& v) { onGga(v); };
    callbacks.on_rmc = [this](const rsf::RmcSentence& v) { onRmc(v); };
    callbacks.on_zda = [this](const rsf::ZdaSentence& v) { onZda(v); };
    callbacks.on_stability = [this](const rsf::TextStatus& v, rsf::StabilityState) {
      onTextStatus(v);
    };
    callbacks.on_source = [this](const rsf::TextStatus& v, const rsf::SourceInfo&) {
      onTextStatus(v);
    };
    callbacks.on_diagnostics = [this](const rsf::Diagnostics& v) { onDiagnostics(v); };
    callbacks.on_log = [this](rsf::LogLevel level, const std::string& message) {
      onLog(level, message);
    };
    callbacks.on_connection_state = [this](rsf::ConnectionState state) {
      RCLCPP_INFO(get_logger(), "sensor connection: %s", rsf::toString(state));
    };
    return callbacks;
  }

  // ******************** Sensor data ********************

  // Builds the message here, on the receive thread, and publishes it on the
  // publishing thread. Only the publish is deferred, so message order per topic
  // is preserved.
  template <typename PublisherT, typename MsgT>
  void enqueue(PublishLane* lane, const PublisherT& publisher, std::unique_ptr<MsgT> msg) {
    lane->post([publisher, message = std::move(msg)]() mutable {
      publisher->publish(std::move(*message));
    });
  }

  // True once per `odom_decimation` samples of this source.
  //
  // Counted rather than timed: the 1 kHz streams arrive in bursts of about
  // fifty, so gating on elapsed wall-clock time discards most of each burst and
  // caps the output near 180 Hz no matter what rate is asked for.
  bool shouldPublishOdometry(rsf::DataType source) {
    if (config_.odom_decimation <= 1) {
      return true;
    }
    const auto index = static_cast<std::size_t>(source);
    if (index >= odom_counter_.size()) {
      return true;
    }

    unsigned& counter = odom_counter_[index];
    if (++counter < static_cast<unsigned>(config_.odom_decimation)) {
      return false;
    }
    counter = 0;
    return true;
  }

  void onOdometry(const rsf::Odometry& odometry) {
    // Recorded before the rate cap, so the cloud is still paired with the most
    // recent pose rather than the most recently published one.
    if (odometry.source == rsf::DataType::kLioOdom && lidar_rate_odom_pub_) {
      std::lock_guard<std::mutex> lock(latest_lio_mutex_);
      latest_lio_odom_ = odometry;
      has_latest_lio_odom_ = true;
    }

    // TF has its own decimation and its own consumers, so it is driven from
    // every sample rather than from the ones that survive the cap.
    if (odometry.source == rsf::DataType::kSwitchOdom) {
      broadcastTf(odometry);
    }

    if (!shouldPublishOdometry(odometry.source)) {
      return;
    }

    auto msg = std::make_unique<nav_msgs::msg::Odometry>();

    switch (odometry.source) {
      case rsf::DataType::kUtmOdom:
        rsf_ros::toOdometryMsg(odometry, config_.frames.utm, config_.frames.lidar, *msg);
        enqueue(utm_odom_lane_, utm_odom_pub_, std::move(msg));
        return;

      case rsf::DataType::kSwitchOdom:
        rsf_ros::toOdometryMsg(odometry, config_.frames.odom, config_.frames.lidar, *msg);
        enqueue(switch_odom_lane_, switch_odom_pub_, std::move(msg));
        return;

      case rsf::DataType::kLioOdom:
        rsf_ros::toOdometryMsg(odometry, config_.frames.odom, config_.frames.lidar, *msg);
        enqueue(lio_odom_lane_, lio_odom_pub_, std::move(msg));
        return;

      default:
        return;
    }
  }

  // One transform per tf_decimation odometry messages: the pose arrives at
  // 1 kHz, far above what a TF tree wants.
  void broadcastTf(const rsf::Odometry& odometry) {
    if (!tf_broadcaster_) {
      return;
    }
    if (++tf_counter_ < static_cast<unsigned>(config_.tf_decimation)) {
      return;
    }
    tf_counter_ = 0;

    auto transform = std::make_unique<geometry_msgs::msg::TransformStamped>();
    rsf_ros::toTransformMsg(odometry, config_.frames.odom, config_.frames.lidar, *transform);
    tf_lane_->post([this, t = std::move(transform)]() mutable {
      tf_broadcaster_->sendTransform(*t);
    });
  }

  void onPointCloud(const rsf::PointCloud& cloud) {
    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    rsf_ros::toPointCloud2Msg(cloud, config_.frames.lidar, *msg);
    enqueue(cloud_lane_, point_cloud_pub_, std::move(msg));

    if (!lidar_rate_odom_pub_) {
      return;
    }

    // Pair the cloud with the most recent pose, so that a consumer gets both
    // sampled at the same rate.
    rsf::Odometry odometry;
    {
      std::lock_guard<std::mutex> lock(latest_lio_mutex_);
      if (!has_latest_lio_odom_) {
        return;
      }
      odometry = latest_lio_odom_;
    }

    auto odometry_msg = std::make_unique<nav_msgs::msg::Odometry>();
    rsf_ros::toOdometryMsg(odometry, config_.frames.odom, config_.frames.lidar, *odometry_msg);
    enqueue(lidar_rate_lane_, lidar_rate_odom_pub_, std::move(odometry_msg));
  }

  void onImu(const rsf::Imu& imu) {
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    rsf_ros::toImuMsg(imu, config_.frames.imu, *msg);
    enqueue(imu_lane_, imu_pub_, std::move(msg));
  }

  void onNavSatFix(const rsf::NavSatFix& fix) {
    auto msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
    rsf_ros::toNavSatFixMsg(fix, config_.frames.gnss, *msg);
    if (fix.source == rsf::DataType::kSwitchFix) {
      enqueue(fix_lane_, switch_fix_pub_, std::move(msg));
    } else {
      enqueue(fix_lane_, fix_pub_, std::move(msg));
    }
  }

  void onGga(const rsf::GgaSentence& gga) {
    auto msg = std::make_unique<nmea_msgs::msg::Gpgga>();
    rsf_ros::toGpggaMsg(gga, config_.frames.gnss, *msg);
    enqueue(slow_lane_, gga_pub_, std::move(msg));
  }

  void onRmc(const rsf::RmcSentence& rmc) {
    auto msg = std::make_unique<nmea_msgs::msg::Gprmc>();
    rsf_ros::toGprmcMsg(rmc, config_.frames.gnss, *msg);
    enqueue(slow_lane_, rmc_pub_, std::move(msg));
  }

  void onZda(const rsf::ZdaSentence& zda) {
    auto msg = std::make_unique<nmea_msgs::msg::Gpzda>();
    rsf_ros::toGpzdaMsg(zda, config_.frames.gnss, *msg);
    enqueue(slow_lane_, zda_pub_, std::move(msg));
  }

  // The four state/type strings arrive at 1 kHz each but describe a state that
  // changes every few seconds, so forwarding all 4000 a second is almost pure
  // overhead. Unless told otherwise, publish when the text changes and
  // otherwise only often enough that a subscriber joining late still gets the
  // current value promptly.
  bool shouldPublishText(rsf::DataType source, const std::string& text) {
    if (!config_.publish_text_on_change_only) {
      return true;
    }
    const auto index = static_cast<std::size_t>(source);
    if (index >= last_text_.size()) {
      return true;
    }

    const auto now = std::chrono::steady_clock::now();
    TextState& state = last_text_[index];
    if (state.valid && state.text == text) {
      if (config_.text_keepalive_hz <= 0.0) {
        return false;
      }
      const auto period =
          std::chrono::duration<double>(1.0 / config_.text_keepalive_hz);
      if (now - state.sent < std::chrono::duration_cast<std::chrono::steady_clock::duration>(period)) {
        return false;
      }
    }

    state.text = text;
    state.valid = true;
    state.sent = now;
    return true;
  }

  void onTextStatus(const rsf::TextStatus& status) {
    if (!shouldPublishText(status.source, status.text)) {
      return;
    }

    auto msg = std::make_unique<std_msgs::msg::String>();
    rsf_ros::toStringMsg(status.text, *msg);

    switch (status.source) {
      case rsf::DataType::kSwitchOdomState:
        enqueue(slow_lane_, switch_odom_state_pub_, std::move(msg));
        break;
      case rsf::DataType::kSwitchOdomType:
        enqueue(slow_lane_, switch_odom_type_pub_, std::move(msg));
        break;
      case rsf::DataType::kSwitchFixState:
        enqueue(slow_lane_, switch_fix_state_pub_, std::move(msg));
        break;
      case rsf::DataType::kSwitchFixType:
        enqueue(slow_lane_, switch_fix_type_pub_, std::move(msg));
        break;
      default:
        break;
    }
  }

  void onDiagnostics(const rsf::Diagnostics& diagnostics) {
    auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
    rsf_ros::toDiagnosticArrayMsg(diagnostics, config_.diagnostic_status_name, *msg);
    enqueue(slow_lane_, diagnostics_pub_, std::move(msg));
  }

  void reportDrops() {
    for (const auto& lane : lanes_) {
      const PublishLane::Report report = lane->takeReport();
      if (report.dropped_since == 0) {
        continue;
      }
      RCLCPP_WARN(get_logger(),
                  "topic '%s': %" PRIu64
                  " samples discarded (queue %zu of %d, slowest publish %.0f ms). Its subscriber "
                  "is not keeping up; other topics are unaffected.",
                  report.name.c_str(), report.dropped_since, report.depth,
                  config_.publish_queue_size, report.worst_publish_seconds * 1000.0);
    }
  }

  void onLog(rsf::LogLevel level, const std::string& message) {
    switch (level) {
      case rsf::LogLevel::kDebug:
        RCLCPP_DEBUG(get_logger(), "%s", message.c_str());
        break;
      case rsf::LogLevel::kInfo:
        RCLCPP_INFO(get_logger(), "%s", message.c_str());
        break;
      case rsf::LogLevel::kWarn:
        RCLCPP_WARN(get_logger(), "%s", message.c_str());
        break;
      case rsf::LogLevel::kError:
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        break;
    }
  }

  // ******************** Commands ********************

  void onCommand(std::uint8_t value) {
    if (value < static_cast<std::uint8_t>(rsf::CommandType::kStartStreaming) ||
        value > static_cast<std::uint8_t>(rsf::CommandType::kResetRsf)) {
      RCLCPP_WARN(get_logger(), "ignoring unknown command number %u", static_cast<unsigned>(value));
      return;
    }

    const auto command = static_cast<rsf::CommandType>(value);
    RCLCPP_INFO(get_logger(), "sending %s", rsf::toString(command));
    if (!client_->sendCommand(command)) {
      RCLCPP_ERROR(get_logger(), "failed to send %s", rsf::toString(command));
    }
  }

  void onSetIpAddress(const std::string& address) {
    RCLCPP_INFO(get_logger(), "sending SET_IP_ADDRESS %s", address.c_str());
    const auto* payload = reinterpret_cast<const std::uint8_t*>(address.data());
    if (!client_->sendCommand(rsf::CommandType::kSetIpAddress, payload, address.size())) {
      RCLCPP_ERROR(get_logger(), "failed to send SET_IP_ADDRESS");
    }
  }

  // ******************** State ********************

  rsf_ros::DriverConfig config_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr gga_pub_;
  rclcpp::Publisher<nmea_msgs::msg::Gprmc>::SharedPtr rmc_pub_;
  rclcpp::Publisher<nmea_msgs::msg::Gpzda>::SharedPtr zda_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lio_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr switch_fix_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr utm_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr switch_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_rate_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switch_odom_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switch_odom_type_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switch_fix_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switch_fix_type_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr set_ip_address_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  unsigned tf_counter_ = 0;

  std::mutex latest_lio_mutex_;
  rsf::Odometry latest_lio_odom_;
  bool has_latest_lio_odom_ = false;

  struct TextState {
    std::string text;
    std::chrono::steady_clock::time_point sent{};
    bool valid = false;
  };
  // Indexed by DataType; only the four state/type entries are ever used.
  std::array<TextState, rsf::kMaxDataType + 1> last_text_;

  // Indexed by DataType; only the three odometry entries are ever used.
  std::array<unsigned, rsf::kMaxDataType + 1> odom_counter_{};

  rclcpp::TimerBase::SharedPtr report_timer_;

  // Declared after the publishers so that they are destroyed before them: a
  // lane's thread must stop touching a publisher while it still exists.
  std::vector<std::unique_ptr<PublishLane>> lanes_;
  PublishLane* cloud_lane_ = nullptr;
  PublishLane* imu_lane_ = nullptr;
  PublishLane* lio_odom_lane_ = nullptr;
  PublishLane* switch_odom_lane_ = nullptr;
  PublishLane* utm_odom_lane_ = nullptr;
  PublishLane* fix_lane_ = nullptr;
  PublishLane* tf_lane_ = nullptr;
  PublishLane* lidar_rate_lane_ = nullptr;
  PublishLane* slow_lane_ = nullptr;

  // Declared last so that it is destroyed first, stopping the receive thread
  // before the queue it posts to.
  std::unique_ptr<rsf::Client> client_;
};

}  // namespace rsf_node2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rsf_node2::RsfNode>());
  rclcpp::shutdown();
  return 0;
}
