// rsf_node2 - tests for the ROS message conversions.
//
// SPDX-License-Identifier: Apache-2.0
//
// conversions.hpp is written against template parameters rather than concrete
// ROS types, so it can be exercised here with stand-in message structs and no
// ROS installation at all.
//
// The stand-ins are declared twice over: once with a ROS 2 style timestamp
// (sec / nanosec) and once with a ROS 1 style one (sec / nsec). Running the
// same conversions through both is what actually verifies the portability
// claim - if a field name only existed in one distribution, this would not
// compile.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "rsf_ros/conversions.hpp"
#include "rsf_ros/driver_config.hpp"

namespace {

int g_checks = 0;
int g_failures = 0;

void report(bool passed, const char* expression, const char* file, int line) {
  ++g_checks;
  if (!passed) {
    ++g_failures;
    std::printf("FAIL %s:%d: %s\n", file, line, expression);
  }
}

#define CHECK(expression) report((expression), #expression, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tolerance)                                                   \
  report(std::fabs(static_cast<double>(a) - static_cast<double>(b)) <= (tolerance), \
         #a " ~= " #b, __FILE__, __LINE__)

// ******************** Stand-in messages ********************

// builtin_interfaces/Time as generated for ROS 2.
struct Ros2Stamp {
  std::int32_t sec = 0;
  std::uint32_t nanosec = 0;
};

// ros::Time as used by ROS 1.
struct Ros1Stamp {
  std::uint32_t sec = 0;
  std::uint32_t nsec = 0;
};

template <typename StampT>
struct Header {
  StampT stamp;
  std::string frame_id;
};

struct Vector3 {
  double x = 0.0, y = 0.0, z = 0.0;
};
struct Quaternion {
  double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
};
struct Pose {
  Vector3 position;
  Quaternion orientation;
};
struct PoseWithCovariance {
  Pose pose;
  std::array<double, 36> covariance{};
};
struct Twist {
  Vector3 linear, angular;
};
struct TwistWithCovariance {
  Twist twist;
  std::array<double, 36> covariance{};
};

template <typename StampT>
struct Odometry {
  Header<StampT> header;
  std::string child_frame_id;
  PoseWithCovariance pose;
  TwistWithCovariance twist;
};

struct Transform {
  Vector3 translation;
  Quaternion rotation;
};

template <typename StampT>
struct TransformStamped {
  Header<StampT> header;
  std::string child_frame_id;
  Transform transform;
};

struct PointField {
  std::string name;
  std::uint32_t offset = 0;
  std::uint8_t datatype = 0;
  std::uint32_t count = 0;
};

template <typename StampT>
struct PointCloud2 {
  Header<StampT> header;
  std::uint32_t height = 0, width = 0;
  std::vector<PointField> fields;
  bool is_bigendian = false;
  std::uint32_t point_step = 0, row_step = 0;
  std::vector<std::uint8_t> data;
  bool is_dense = false;
};

template <typename StampT>
struct Imu {
  Header<StampT> header;
  Quaternion orientation;
  std::array<double, 9> orientation_covariance{};
  Vector3 angular_velocity;
  std::array<double, 9> angular_velocity_covariance{};
  Vector3 linear_acceleration;
  std::array<double, 9> linear_acceleration_covariance{};
};

struct NavSatStatus {
  std::int8_t status = -1;
  std::uint16_t service = 0;
};

template <typename StampT>
struct NavSatFix {
  Header<StampT> header;
  NavSatStatus status;
  double latitude = 0.0, longitude = 0.0, altitude = 0.0;
  std::array<double, 9> position_covariance{};
  std::uint8_t position_covariance_type = 0;
};

template <typename StampT>
struct Gpgga {
  Header<StampT> header;
  std::string message_id;
  double utc_seconds = 0.0;
  double lat = 0.0, lon = 0.0;
  std::string lat_dir, lon_dir;
  std::uint32_t gps_qual = 0, num_sats = 0;
  float hdop = 0.0f, alt = 0.0f;
  std::string altitude_units;
  float undulation = 0.0f;
  std::string undulation_units;
  std::uint32_t diff_age = 0;
  std::string station_id;
};

template <typename StampT>
struct Gprmc {
  Header<StampT> header;
  std::string message_id;
  double utc_seconds = 0.0;
  std::string position_status;
  double lat = 0.0, lon = 0.0;
  std::string lat_dir, lon_dir;
  float speed = 0.0f, track = 0.0f;
  std::string date;
  float mag_var = 0.0f;
  std::string mag_var_direction, mode_indicator;
};

template <typename StampT>
struct Gpzda {
  Header<StampT> header;
  std::string message_id;
  std::uint32_t utc_seconds = 0;
  std::uint8_t day = 0, month = 0;
  std::uint16_t year = 0;
  std::int8_t hour_offset_gmt = 0;
  std::uint8_t minute_offset_gmt = 0;
};

struct KeyValue {
  std::string key, value;
};

struct DiagnosticStatus {
  std::uint8_t level = 0;
  std::string name, message, hardware_id;
  std::vector<KeyValue> values;
};

template <typename StampT>
struct DiagnosticArray {
  Header<StampT> header;
  std::vector<DiagnosticStatus> status;
};

struct StringMsg {
  std::string data;
};

// ******************** Sample inputs ********************

rsf::Timestamp sampleStamp() {
  rsf::Timestamp stamp;
  stamp.sec = 1774483200;
  stamp.nsec = 123456789;
  return stamp;
}

rsf::Odometry sampleOdometry() {
  rsf::Odometry odometry;
  odometry.stamp = sampleStamp();
  odometry.source = rsf::DataType::kSwitchOdom;
  odometry.position = {1.5f, -2.5f, 0.25f};
  odometry.orientation = {0.0f, 0.0f, 0.7071068f, 0.7071068f};
  odometry.linear_velocity = {2.0f, 0.1f, 0.0f};
  odometry.angular_velocity = {0.0f, 0.0f, 0.2f};
  for (std::size_t i = 0; i < 36; ++i) {
    odometry.pose_covariance[i] = static_cast<float>(i);
    odometry.velocity_covariance[i] = static_cast<float>(100 + i);
  }
  return odometry;
}

// ******************** Conversion checks ********************

// Run once per timestamp flavour. Anything that only compiled against one of
// the two distributions would fail to instantiate here.
template <typename StampT>
void checkConversions(const char* flavour) {
  std::printf("  checking with %s timestamps\n", flavour);

  const rsf::Timestamp stamp = sampleStamp();

  // --- Odometry ---
  {
    const rsf::Odometry odometry = sampleOdometry();
    Odometry<StampT> msg;
    rsf_ros::toOdometryMsg(odometry, "rsf_odom", "rsf_hokuyo3d", msg);

    CHECK(msg.header.frame_id == "rsf_odom");
    CHECK(msg.child_frame_id == "rsf_hokuyo3d");
    CHECK(static_cast<std::uint32_t>(msg.header.stamp.sec) == stamp.sec);
    CHECK_NEAR(msg.pose.pose.position.x, 1.5, 1e-6);
    CHECK_NEAR(msg.pose.pose.position.y, -2.5, 1e-6);
    CHECK_NEAR(msg.pose.pose.orientation.w, 0.7071068, 1e-6);
    CHECK_NEAR(msg.twist.twist.linear.x, 2.0, 1e-6);
    CHECK_NEAR(msg.twist.twist.angular.z, 0.2, 1e-6);
    CHECK_NEAR(msg.pose.covariance[35], 35.0, 1e-6);
    CHECK_NEAR(msg.twist.covariance[35], 135.0, 1e-6);
  }

  // --- TransformStamped ---
  {
    const rsf::Odometry odometry = sampleOdometry();
    TransformStamped<StampT> msg;
    rsf_ros::toTransformMsg(odometry, "rsf_odom", "rsf_hokuyo3d", msg);

    CHECK(msg.header.frame_id == "rsf_odom");
    CHECK(msg.child_frame_id == "rsf_hokuyo3d");
    CHECK_NEAR(msg.transform.translation.x, 1.5, 1e-6);
    CHECK_NEAR(msg.transform.rotation.z, 0.7071068, 1e-6);
  }

  // --- PointCloud2 ---
  {
    rsf::PointCloud cloud;
    cloud.stamp = stamp;
    cloud.points.resize(3);
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
      cloud.points[i].x = static_cast<float>(i);
      cloud.points[i].y = static_cast<float>(i) + 0.5f;
      cloud.points[i].z = static_cast<float>(i) + 0.25f;
      cloud.points[i].intensity = static_cast<float>(10 * i);
      cloud.points[i].sec = 1774483200u + static_cast<std::uint32_t>(i);
      cloud.points[i].nsec = 500u;
    }

    PointCloud2<StampT> msg;
    rsf_ros::toPointCloud2Msg(cloud, "rsf_hokuyo3d", msg);

    CHECK(msg.height == 1);
    CHECK(msg.width == 3);
    CHECK(msg.point_step == 24);
    CHECK(msg.row_step == 72);
    CHECK(msg.data.size() == 72);
    CHECK(msg.is_dense);
    CHECK(!msg.is_bigendian);

    CHECK(msg.fields.size() == 6);
    CHECK(msg.fields[0].name == "x" && msg.fields[0].offset == 0);
    CHECK(msg.fields[3].name == "intensity" && msg.fields[3].offset == 12);
    CHECK(msg.fields[4].name == "sec" && msg.fields[4].offset == 16);
    CHECK(msg.fields[5].name == "nsec" && msg.fields[5].offset == 20);
    CHECK(msg.fields[0].datatype == rsf_ros::point_field::kFloat32);
    CHECK(msg.fields[4].datatype == rsf_ros::point_field::kUint32);
    CHECK(msg.fields[0].count == 1);

    // The blob has to be readable back as the points that went in.
    const std::uint8_t* third = msg.data.data() + 2 * 24;
    CHECK_NEAR(rsf::readF32(third + 0), 2.0, 1e-6);
    CHECK_NEAR(rsf::readF32(third + 4), 2.5, 1e-6);
    CHECK_NEAR(rsf::readF32(third + 12), 20.0, 1e-6);
    CHECK(rsf::readU32(third + 16) == 1774483202u);
    CHECK(rsf::readU32(third + 20) == 500u);

    // Restamping onto another clock has to move the per-point times with the
    // header. Leaving them behind would put two clocks in one message and
    // silently wreck any motion compensation reading them.
    PointCloud2<StampT> shifted;
    const std::int64_t shift_ns = 700000000;  // +0.7 s, the offset measured here
    rsf_ros::toPointCloud2Msg(cloud, "rsf_hokuyo3d", shifted, shift_ns);

    const std::uint8_t* first = shifted.data.data();
    CHECK(rsf::readU32(first + 16) == 1774483200u);
    CHECK(rsf::readU32(first + 20) == 500u + 700000000u);

    // The third point's nanoseconds carry into the next second.
    const std::uint8_t* moved_third = shifted.data.data() + 2 * 24;
    CHECK(rsf::readU32(moved_third + 16) == 1774483202u);
    CHECK(rsf::readU32(moved_third + 20) == 700000500u);

    // Everything that is not a timestamp survives the shift untouched.
    CHECK_NEAR(rsf::readF32(moved_third + 0), 2.0, 1e-6);
    CHECK_NEAR(rsf::readF32(moved_third + 12), 20.0, 1e-6);

    // A shift that crosses a second boundary must carry properly.
    PointCloud2<StampT> carried;
    rsf_ros::toPointCloud2Msg(cloud, "rsf_hokuyo3d", carried, 1500000000);  // +1.5 s
    const std::uint8_t* carried_first = carried.data.data();
    CHECK(rsf::readU32(carried_first + 16) == 1774483201u);
    CHECK(rsf::readU32(carried_first + 20) == 500000500u);
  }

  // --- Imu ---
  {
    rsf::Imu imu;
    imu.stamp = stamp;
    imu.angular_velocity = {0.1f, 0.2f, 0.3f};
    imu.linear_acceleration = {0.0f, 0.0f, 9.81f};
    imu.angular_velocity_covariance[4] = 0.5f;

    Imu<StampT> msg;
    rsf_ros::toImuMsg(imu, "rsf_hokuyo3d_imu", msg);

    CHECK(msg.header.frame_id == "rsf_hokuyo3d_imu");
    CHECK_NEAR(msg.angular_velocity.z, 0.3, 1e-6);
    CHECK_NEAR(msg.linear_acceleration.z, 9.81, 1e-5);
    CHECK_NEAR(msg.angular_velocity_covariance[4], 0.5, 1e-6);
  }

  // --- NavSatFix ---
  {
    rsf::NavSatFix fix;
    fix.stamp = stamp;
    fix.latitude = 34.6937;
    fix.longitude = 135.5023;
    fix.altitude = 40.5;
    fix.status = static_cast<std::int8_t>(rsf::FixStatus::kGbasFix);
    fix.service = rsf::kServiceGps | rsf::kServiceGlonass;
    fix.position_covariance_type = 2;
    fix.position_covariance[8] = 9.0;

    NavSatFix<StampT> msg;
    rsf_ros::toNavSatFixMsg(fix, "rsf_gnss", msg);

    CHECK(msg.header.frame_id == "rsf_gnss");
    CHECK_NEAR(msg.latitude, 34.6937, 1e-9);
    CHECK_NEAR(msg.longitude, 135.5023, 1e-9);
    CHECK(msg.status.status == 2);
    CHECK(msg.status.service == 3);
    CHECK(msg.position_covariance_type == 2);
    CHECK_NEAR(msg.position_covariance[8], 9.0, 1e-9);
  }

  // --- Gpgga ---
  {
    rsf::GgaSentence gga;
    gga.stamp = stamp;
    gga.message_id = "$GPGGA";
    gga.utc_second = 123519.0;
    gga.latitude = 3441.6222;
    gga.longitude = 13530.138;
    gga.latitude_direction = 'N';
    gga.longitude_direction = 'E';
    gga.altitude_units = 'M';
    gga.undulation_units = 'M';
    gga.position_status = 4;
    gga.num_satellites = 21;
    gga.hdop = 0.8f;
    gga.altitude = 40.5f;
    gga.undulation = 36.7f;
    gga.diff_age = 3;
    gga.station_id = "0123";

    Gpgga<StampT> msg;
    rsf_ros::toGpggaMsg(gga, "rsf_gnss", msg);

    CHECK(msg.message_id == "$GPGGA");
    // The transmitted ddmm.mmmm form is preserved, not converted.
    CHECK_NEAR(msg.lat, 3441.6222, 1e-6);
    CHECK(msg.lat_dir == "N");
    CHECK(msg.lon_dir == "E");
    CHECK(msg.altitude_units == "M");
    CHECK(msg.undulation_units == "M");
    CHECK(msg.gps_qual == 4);
    CHECK(msg.num_sats == 21);
    CHECK(msg.diff_age == 3);
    CHECK(msg.station_id == "0123");
    CHECK_NEAR(msg.undulation, 36.7, 1e-5);
  }

  // --- Gprmc ---
  {
    rsf::RmcSentence rmc;
    rmc.stamp = stamp;
    rmc.message_id = "$GPRMC";
    rmc.utc_second = 123519.0;
    rmc.latitude = 34.6937;
    rmc.longitude = 135.5023;
    rmc.latitude_direction = 'N';
    rmc.longitude_direction = 'E';
    rmc.speed = 3.888;
    rmc.track = 92.5;
    rmc.date = "120326";
    rmc.magnetic_variation = 7.5;
    rmc.magnetic_variation_direction = 'W';
    rmc.status = 'V';
    rmc.mode = 'R';

    Gprmc<StampT> msg;
    rsf_ros::toGprmcMsg(rmc, "rsf_gnss", msg);

    CHECK(msg.message_id == "$GPRMC");
    CHECK(msg.position_status == "V");
    CHECK(msg.mode_indicator == "R");
    CHECK(msg.mag_var_direction == "W");
    CHECK(msg.date == "120326");
    CHECK_NEAR(msg.speed, 3.888, 1e-5);
    CHECK_NEAR(msg.track, 92.5, 1e-5);
    // RMC transmits degrees, so no conversion is applied here either.
    CHECK_NEAR(msg.lat, 34.6937, 1e-9);
  }

  // --- Gpzda ---
  {
    rsf::ZdaSentence zda;
    zda.stamp = stamp;
    zda.message_id = "$GPZDA";
    zda.utc_second = 45296;
    zda.day = 12;
    zda.month = 3;
    zda.year = 2026;

    Gpzda<StampT> msg;
    rsf_ros::toGpzdaMsg(zda, "rsf_gnss", msg);

    CHECK(msg.message_id == "$GPZDA");
    CHECK(msg.utc_seconds == 45296u);
    CHECK(msg.day == 12);
    CHECK(msg.month == 3);
    CHECK(msg.year == 2026);
    CHECK(msg.hour_offset_gmt == 0);
  }

  // --- DiagnosticArray ---
  {
    rsf::Diagnostics diagnostics;
    diagnostics.stamp = stamp;
    diagnostics.ip_address = "192.168.0.100";
    diagnostics.port = 10940;
    diagnostics.product_name = "RSF-X001";
    diagnostics.software_major = 1;
    diagnostics.software_minor = 0;
    diagnostics.software_patch = 3;
    diagnostics.device_id = "SN000042";
    diagnostics.device_status = 0;
    diagnostics.device_temperature = 47;
    diagnostics.cpu_usage = 33;
    diagnostics.elapsed_time = 1234;
    diagnostics.odometry_state = 2;
    diagnostics.odometry_type = 1;
    diagnostics.gnss_state = 3;
    diagnostics.gnss_type = 2;

    DiagnosticArray<StampT> msg;
    rsf_ros::toDiagnosticArrayMsg(diagnostics, "rsf_device", msg);

    CHECK(msg.status.size() == 1);
    CHECK(msg.status[0].name == "rsf_device");
    CHECK(msg.status[0].hardware_id == "SN000042");
    CHECK(msg.status[0].level == rsf_ros::diagnostic_level::kOk);

    const auto find = [&msg](const std::string& key) {
      for (const KeyValue& entry : msg.status[0].values) {
        if (entry.key == key) {
          return entry.value;
        }
      }
      return std::string("<missing>");
    };
    CHECK(find("ip_address") == "192.168.0.100");
    CHECK(find("ip_port") == "10940");
    CHECK(find("product_name") == "RSF-X001");
    CHECK(find("firmware_version") == "1.0.3");
    CHECK(find("device_id") == "SN000042");
    CHECK(find("cpu_usage") == "33");
    CHECK(find("elapsed_time") == "1234");
    CHECK(find("odometry_state") == "2");
    // The decoded names are published alongside the raw numbers.
    CHECK(find("odometry_state_text") == "normal");
    CHECK(find("odometry_source") == "LIO");
    CHECK(find("gnss_state_text") == "good");
    CHECK(find("gnss_source") == "GNSS");

    // A fault has to raise the level above OK.
    diagnostics.device_status = 1;
    rsf_ros::toDiagnosticArrayMsg(diagnostics, "rsf_device", msg);
    CHECK(msg.status[0].level == rsf_ros::diagnostic_level::kWarn);
  }

  // --- String ---
  {
    StringMsg msg;
    rsf_ros::toStringMsg("LIO (switch)", msg);
    CHECK(msg.data == "LIO (switch)");
  }
}

// The timestamp is the only field that differs between the distributions, so it
// gets a dedicated check on top of the shared run above.
void checkTimestamps() {
  rsf::Timestamp time;
  time.sec = 1774483200;
  time.nsec = 123456789;

  Ros2Stamp ros2;
  rsf_ros::toRosTime(time, ros2);
  CHECK(ros2.sec == 1774483200);
  CHECK(ros2.nanosec == 123456789u);

  Ros1Stamp ros1;
  rsf_ros::toRosTime(time, ros1);
  CHECK(ros1.sec == 1774483200u);
  CHECK(ros1.nsec == 123456789u);
}

void checkCharToString() {
  CHECK(rsf_ros::charToString('N') == "N");
  // An absent character must become an empty string, not a NUL string.
  CHECK(rsf_ros::charToString('\0').empty());
}

void checkDriverConfig() {
  rsf_ros::DriverConfig config;
  CHECK(config.validate().empty());

  const rsf::ClientConfig client = config.toClientConfig();
  CHECK(client.host == config.ip_address);
  CHECK(client.port == config.port);

  // The default pins the legacy dialect rather than detecting it, so a
  // communication fault shows up as a fault instead of being reinterpreted.
  CHECK(config.wire_layout == "legacy");
  CHECK(!client.parser.auto_detect_layout);
  CHECK(client.parser.layout == rsf::legacyLayout());

  config.wire_layout = "spec";
  CHECK(config.toClientConfig().parser.layout == rsf::specificationLayout());
  CHECK(!config.toClientConfig().parser.auto_detect_layout);

  config.wire_layout = "auto";
  CHECK(config.toClientConfig().parser.auto_detect_layout);

  // Bad values must be reported rather than silently accepted.
  config.wire_layout = "nonsense";
  CHECK(!config.validate().empty());

  config = rsf_ros::DriverConfig();
  config.port = 0;
  CHECK(!config.validate().empty());

  config = rsf_ros::DriverConfig();
  config.ip_address.clear();
  CHECK(!config.validate().empty());

  config = rsf_ros::DriverConfig();
  config.tf_decimation = 0;
  CHECK(!config.validate().empty());
}

}  // namespace

int main() {
  checkTimestamps();
  checkCharToString();
  checkDriverConfig();
  checkConversions<Ros2Stamp>("ROS 2 style");
  checkConversions<Ros1Stamp>("ROS 1 style");

  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
