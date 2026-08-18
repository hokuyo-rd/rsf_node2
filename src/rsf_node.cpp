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
// callbacks below are invoked from it. rclcpp publishers are safe to use from
// any thread, and the callbacks do no blocking work beyond publishing.

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

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rsf/rsf.hpp"
#include "rsf_ros/conversions.hpp"
#include "rsf_ros/driver_config.hpp"

namespace rsf_node2 {

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

    RCLCPP_INFO(get_logger(), "connecting to RSF-X001 at %s:%d", config_.ip_address.c_str(),
                config_.port);

    client_ = std::make_unique<rsf::Client>(config_.toClientConfig(), makeCallbacks());
    client_->start();
  }

  ~RsfNode() override {
    // Stop receiving before the publishers go away.
    if (client_) {
      client_->stop();
      client_->disconnect();
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

  void onOdometry(const rsf::Odometry& odometry) {
    nav_msgs::msg::Odometry msg;

    switch (odometry.source) {
      case rsf::DataType::kUtmOdom:
        rsf_ros::toOdometryMsg(odometry, config_.frames.utm, config_.frames.lidar, msg);
        utm_odom_pub_->publish(msg);
        return;

      case rsf::DataType::kSwitchOdom:
        rsf_ros::toOdometryMsg(odometry, config_.frames.odom, config_.frames.lidar, msg);
        switch_odom_pub_->publish(msg);
        broadcastTf(odometry);
        return;

      case rsf::DataType::kLioOdom:
        rsf_ros::toOdometryMsg(odometry, config_.frames.odom, config_.frames.lidar, msg);
        lio_odom_pub_->publish(msg);
        if (lidar_rate_odom_pub_) {
          std::lock_guard<std::mutex> lock(latest_lio_mutex_);
          latest_lio_odom_ = odometry;
          has_latest_lio_odom_ = true;
        }
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

    geometry_msgs::msg::TransformStamped transform;
    rsf_ros::toTransformMsg(odometry, config_.frames.odom, config_.frames.lidar, transform);
    tf_broadcaster_->sendTransform(transform);
  }

  void onPointCloud(const rsf::PointCloud& cloud) {
    sensor_msgs::msg::PointCloud2 msg;
    rsf_ros::toPointCloud2Msg(cloud, config_.frames.lidar, msg);
    point_cloud_pub_->publish(msg);

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

    nav_msgs::msg::Odometry odometry_msg;
    rsf_ros::toOdometryMsg(odometry, config_.frames.odom, config_.frames.lidar, odometry_msg);
    lidar_rate_odom_pub_->publish(odometry_msg);
  }

  void onImu(const rsf::Imu& imu) {
    sensor_msgs::msg::Imu msg;
    rsf_ros::toImuMsg(imu, config_.frames.imu, msg);
    imu_pub_->publish(msg);
  }

  void onNavSatFix(const rsf::NavSatFix& fix) {
    sensor_msgs::msg::NavSatFix msg;
    rsf_ros::toNavSatFixMsg(fix, config_.frames.gnss, msg);
    if (fix.source == rsf::DataType::kSwitchFix) {
      switch_fix_pub_->publish(msg);
    } else {
      fix_pub_->publish(msg);
    }
  }

  void onGga(const rsf::GgaSentence& gga) {
    nmea_msgs::msg::Gpgga msg;
    rsf_ros::toGpggaMsg(gga, config_.frames.gnss, msg);
    gga_pub_->publish(msg);
  }

  void onRmc(const rsf::RmcSentence& rmc) {
    nmea_msgs::msg::Gprmc msg;
    rsf_ros::toGprmcMsg(rmc, config_.frames.gnss, msg);
    rmc_pub_->publish(msg);
  }

  void onZda(const rsf::ZdaSentence& zda) {
    nmea_msgs::msg::Gpzda msg;
    rsf_ros::toGpzdaMsg(zda, config_.frames.gnss, msg);
    zda_pub_->publish(msg);
  }

  void onTextStatus(const rsf::TextStatus& status) {
    std_msgs::msg::String msg;
    rsf_ros::toStringMsg(status.text, msg);

    switch (status.source) {
      case rsf::DataType::kSwitchOdomState:
        switch_odom_state_pub_->publish(msg);
        break;
      case rsf::DataType::kSwitchOdomType:
        switch_odom_type_pub_->publish(msg);
        break;
      case rsf::DataType::kSwitchFixState:
        switch_fix_state_pub_->publish(msg);
        break;
      case rsf::DataType::kSwitchFixType:
        switch_fix_type_pub_->publish(msg);
        break;
      default:
        break;
    }
  }

  void onDiagnostics(const rsf::Diagnostics& diagnostics) {
    diagnostic_msgs::msg::DiagnosticArray msg;
    rsf_ros::toDiagnosticArrayMsg(diagnostics, config_.diagnostic_status_name, msg);
    diagnostics_pub_->publish(msg);
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

  // Declared last so that it is destroyed first, stopping the receive thread
  // before the publishers it uses.
  std::unique_ptr<rsf::Client> client_;
};

}  // namespace rsf_node2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rsf_node2::RsfNode>());
  rclcpp::shutdown();
  return 0;
}
