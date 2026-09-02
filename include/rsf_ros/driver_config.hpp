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

  // Which clock stamps the published messages.
  //
  // "sensor"  the timestamp the sensor put on the wire - the true acquisition
  //           instant, correct when the sensor clock is synchronised here.
  //
  // "receive" the ROS clock: the sensor's timestamps moved onto it by one
  //           continuously measured offset. Use this when the sensor clock is
  //           not disciplined - without a GNSS fix this unit was measured about
  //           0.7 s behind the PC, which no Nav2 costmap or transform tolerance
  //           accepts.
  //
  //           The offset is shared by every stream on purpose. Stamping each
  //           message with its own arrival time instead reorders the streams
  //           against each other: the sensor stamps a cloud at acquisition,
  //           before the odometry it goes on to produce, while arrival time puts
  //           it after. Measured here, doing that took tf2 lookups on the point
  //           cloud from 6 % failing to 99 %.
  std::string stamp_source = "sensor";

  bool broadcast_tf = true;
  // One transform per this many odometry messages, so 2 gives 500 Hz out of the
  // 1 kHz stream. Bounded from both sides, both measured on hardware:
  //
  //   too slow - tf2 interpolates but never extrapolates, so anything stamped
  //   after the newest transform cannot be looked up. At 20 Hz, 6.4 % of point
  //   cloud lookups failed.
  //
  //   too fast - /tf has RELIABLE subscribers, and past what they absorb the
  //   publish lane discards transforms while each subscriber's tf2 listener
  //   falls behind as well. At 1000 Hz the lane overflowed, the largest gap grew
  //   from 0.004 s to 0.155 s, and failures rose to 10.9 %.
  //
  // 100 Hz (10) and 500 Hz (2) measured the same in between. 2 is chosen for the
  // headroom; 10 costs slightly less on a CPU bound machine. Do not set 1.
  int tf_decimation = 2;

  bool publish_lidar_rate_odom = true;

  // Publisher queue depth. The 1 kHz streams need more than the usual 10.
  int queue_size = 100;

  // Depth of the node's own hand-off queue between the sensor receive thread
  // and the thread that calls publish(). A rclcpp publisher can block - a
  // RELIABLE writer whose history is full waits for its subscribers - and with
  // every topic published inline on the receive thread that stalled the whole
  // node and backed the sensor socket up. Roughly 0.3 s of traffic at the
  // combined ~7 kHz output rate.
  int publish_queue_size = 2000;

  // The four state/type strings of sections 3.8.8/3.8.9 arrive at 1 kHz but
  // change only every few seconds, so by default they are published when the
  // text changes and otherwise at `text_keepalive_hz`, which takes about 4000
  // messages a second out of the graph. Set false to forward every one.
  bool publish_text_on_change_only = true;
  double text_keepalive_hz = 1.0;

  // One odometry message is published per this many received, per source.
  // 1 publishes every sample, 10 gives roughly 100 Hz out of the 1 kHz stream.
  //
  // Counting samples rather than watching the clock is deliberate: the sensor
  // delivers the 1 kHz streams in bursts of about fifty, so a wall-clock rate
  // limiter throws most of each burst away and lands near 180 Hz whatever it is
  // set to. Decimation is exact and independent of how the data is batched,
  // which is also why tf_decimation is expressed the same way.
  //
  // Applies to lio_odom, switch_odom and utm_odom. TF has its own decimation,
  // and IMU is left alone.
  int odom_decimation = 1;

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

    // Deliberately left at kSensor. The node does its own mapping onto the ROS
    // clock, which honours use_sim_time; letting the library shift as well
    // would apply the correction twice. The library's own stamp_source is for
    // consumers that use it without ROS, such as rsf_driver and the recorder.
    client.stamp_source = rsf::StampSource::kSensor;
    return client;
  }

  // "offset" is accepted as a synonym: the correction has always been an offset,
  // and the name says what it does rather than when it is measured.
  bool useReceiveStamp() const { return stamp_source == "receive" || stamp_source == "offset"; }

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
    if (!useReceiveStamp() && stamp_source != "sensor") {
      return "stamp_source must be one of: sensor, receive";
    }
    if (tf_decimation < 1) {
      return "tf_decimation must be at least 1";
    }
    if (queue_size < 1) {
      return "queue_size must be at least 1";
    }
    if (publish_queue_size < 1) {
      return "publish_queue_size must be at least 1";
    }
    if (text_keepalive_hz < 0.0) {
      return "text_keepalive_hz must not be negative";
    }
    if (odom_decimation < 1) {
      return "odom_decimation must be at least 1";
    }
    return std::string();
  }
};

}  // namespace rsf_ros
