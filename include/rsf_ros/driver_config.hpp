// rsf_ros - ROS glue shared by the RSF-X001 drivers.
//
// SPDX-License-Identifier: Apache-2.0
//
// Everything the driver is configured with, as plain data.
//
// Holding the settings in a struct instead of reading them straight from the
// node keeps the defaults in one readable place and lets a ROS 1 node fill the
// same structure through its own parameter API.

#pragma once

#include <string>

#include "rsf/client.hpp"

namespace rsf_ros {

// Topic names, one per data type of table 3-3.
struct TopicConfig {
  std::string fix = "/rsf/nav_sat_fix";                   // FIX
  std::string gga = "/rsf/gpgga";                         // GGA
  std::string rmc = "/rsf/gprmc";                         // RMC
  std::string zda = "/rsf/gpzda";                         // ZDA
  std::string point_cloud = "/rsf/hokuyo_cloud2";         // HOKUYO_CLOUD2
  std::string imu = "/rsf/imu";                           // IMU
  std::string lio_odom = "/rsf/lio_imu_rate_odom";        // LIO_ODOM
  std::string switch_fix = "/rsf/rsf_fix";                // SWITCH_FIX
  std::string utm_odom = "/rsf/utm_coord_odom";           // UTM_ODOM
  std::string switch_odom = "/rsf/rsf_odom";              // SWITCH_ODOM
  std::string switch_odom_state = "/rsf/rsf_odom_state";  // SWITCH_ODOM_STATE
  std::string switch_odom_type = "/rsf/rsf_odom_type";    // SWITCH_ODOM_TYPE
  std::string switch_fix_state = "/rsf/rsf_fix_state";    // SWITCH_FIX_STATE
  std::string switch_fix_type = "/rsf/rsf_fix_type";      // SWITCH_FIX_TYPE
  std::string diagnostics = "/rsf/diagnostics";           // DIAGNOSTICS_ARRAY

  // The latest LIO odometry, re-published at point cloud rate so that a
  // consumer of the cloud has a pose sampled at the same moment.
  std::string lidar_rate_odom = "/rsf/lio_lidar_rate_odom";

  // Subscribed: a command number from table 3-2 (1..5).
  std::string command = "/rsf/cmd_to_spel";
  // Subscribed: a new sensor IP address. Vendor extension, see DriverConfig.
  std::string set_ip_address = "/rsf/ip_address";
};

// TF frame names.
struct FrameConfig {
  std::string odom = "rsf_odom";          // parent of the odometry poses
  std::string utm = "rsf_utm/utm";        // parent of the UTM poses
  std::string lidar = "rsf_hokuyo3d";     // the sensor body
  std::string imu = "rsf_hokuyo3d_imu";   // rotated relative to the LiDAR
  std::string gnss = "rsf_gnss";
};

struct DriverConfig {
  std::string ip_address = rsf::kDefaultHost;
  int port = rsf::kDefaultPort;

  // Commands issued once the connection comes up, per the state diagram of
  // section 3.4. Streaming has to be started before position estimation.
  bool start_streaming_on_connect = true;
  bool start_rsf_on_connect = true;

  // The wire dialect: "auto" detects it from the first frames, "spec" and
  // "legacy" pin it. See the rsf_library README on why this exists.
  std::string wire_layout = "legacy";

  bool broadcast_tf = true;
  // Odometry arrives at 1 kHz, which is far more than a TF tree needs.
  // One transform is broadcast per this many odometry messages.
  int tf_decimation = 50;

  bool publish_lidar_rate_odom = true;

  // Publisher queue depth. The 1 kHz streams need more than the usual 10.
  int queue_size = 100;

  // Name reported in diagnostic_msgs/DiagnosticStatus::name.
  std::string diagnostic_status_name = "rsf_device";

  // SET_IP_ADDRESS is not part of C-42-04636; it is accepted by some firmware
  // builds. Leave this off unless the sensor is known to support it.
  bool enable_set_ip_address = false;

  TopicConfig topics;
  FrameConfig frames;

  // Translates into the settings the client library expects.
  rsf::ClientConfig toClientConfig() const {
    rsf::ClientConfig client;
    client.host = ip_address;
    client.port = static_cast<std::uint16_t>(port);
    client.start_streaming_on_connect = start_streaming_on_connect;
    client.start_rsf_on_connect = start_rsf_on_connect;

    if (wire_layout == "spec") {
      client.parser.layout = rsf::specificationLayout();
      client.parser.auto_detect_layout = false;
    } else if (wire_layout == "legacy") {
      client.parser.layout = rsf::legacyLayout();
      client.parser.auto_detect_layout = false;
    }
    return client;
  }

  // Returns an empty string when the configuration is usable, or a description
  // of the problem otherwise.
  std::string validate() const {
    if (ip_address.empty()) {
      return "ip_address must not be empty";
    }
    if (port <= 0 || port > 65535) {
      return "port must be in the range 1..65535";
    }
    if (wire_layout != "auto" && wire_layout != "spec" && wire_layout != "legacy") {
      return "wire_layout must be one of: auto, spec, legacy";
    }
    if (tf_decimation < 1) {
      return "tf_decimation must be at least 1";
    }
    if (queue_size < 1) {
      return "queue_size must be at least 1";
    }
    return std::string();
  }
};

}  // namespace rsf_ros
