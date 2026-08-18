// rsf_ros - ROS glue shared by the RSF-X001 drivers.
//
// SPDX-License-Identifier: Apache-2.0
//
// Conversion from the library's plain data structures to ROS messages.
//
// Every function is a template over the message type rather than being written
// against sensor_msgs::msg::X, so the same header serves a ROS 2 node
// (sensor_msgs::msg::NavSatFix) and a ROS 1 node (sensor_msgs::NavSatFix). The
// field names are identical between the two; ros_compat.hpp absorbs the only
// real difference, which is the timestamp.
//
// Nothing here includes a ROS header, which is also what lets the unit test
// exercise these functions against stand-in message structs.

#pragma once

#include <cstdint>
#include <string>

#include "rsf/types.hpp"
#include "rsf_ros/ros_compat.hpp"

namespace rsf_ros {

// ******************** Odometry ********************

// nav_msgs/Odometry. `frame_id` is the reference frame the pose is expressed
// in, `child_frame_id` the frame being located (the sensor).
template <typename OdometryMsg>
void toOdometryMsg(const rsf::Odometry& odometry, const std::string& frame_id,
                   const std::string& child_frame_id, OdometryMsg& msg) {
  fillHeader(odometry.stamp, frame_id, msg.header);
  msg.child_frame_id = child_frame_id;

  msg.pose.pose.position.x = odometry.position.x;
  msg.pose.pose.position.y = odometry.position.y;
  msg.pose.pose.position.z = odometry.position.z;
  msg.pose.pose.orientation.x = odometry.orientation.x;
  msg.pose.pose.orientation.y = odometry.orientation.y;
  msg.pose.pose.orientation.z = odometry.orientation.z;
  msg.pose.pose.orientation.w = odometry.orientation.w;

  msg.twist.twist.linear.x = odometry.linear_velocity.x;
  msg.twist.twist.linear.y = odometry.linear_velocity.y;
  msg.twist.twist.linear.z = odometry.linear_velocity.z;
  msg.twist.twist.angular.x = odometry.angular_velocity.x;
  msg.twist.twist.angular.y = odometry.angular_velocity.y;
  msg.twist.twist.angular.z = odometry.angular_velocity.z;

  // Both are row major 6x6 over (x, y, z, roll, pitch, yaw), so the layout
  // carries across unchanged.
  for (std::size_t i = 0; i < 36; ++i) {
    msg.pose.covariance[i] = odometry.pose_covariance[i];
    msg.twist.covariance[i] = odometry.velocity_covariance[i];
  }
}

// geometry_msgs/TransformStamped, carrying the same pose as toOdometryMsg().
template <typename TransformStampedMsg>
void toTransformMsg(const rsf::Odometry& odometry, const std::string& frame_id,
                    const std::string& child_frame_id, TransformStampedMsg& msg) {
  fillHeader(odometry.stamp, frame_id, msg.header);
  msg.child_frame_id = child_frame_id;

  msg.transform.translation.x = odometry.position.x;
  msg.transform.translation.y = odometry.position.y;
  msg.transform.translation.z = odometry.position.z;
  msg.transform.rotation.x = odometry.orientation.x;
  msg.transform.rotation.y = odometry.orientation.y;
  msg.transform.rotation.z = odometry.orientation.z;
  msg.transform.rotation.w = odometry.orientation.w;
}

// ******************** Point cloud ********************

// Byte layout of one point inside PointCloud2::data, matching the POINTXYZIT
// format of section 3.8.2 so that the payload maps one to one.
namespace point_layout {
constexpr std::uint32_t kOffsetX = 0;
constexpr std::uint32_t kOffsetY = 4;
constexpr std::uint32_t kOffsetZ = 8;
constexpr std::uint32_t kOffsetIntensity = 12;
constexpr std::uint32_t kOffsetSec = 16;
constexpr std::uint32_t kOffsetNsec = 20;
constexpr std::uint32_t kPointStep = 24;
}  // namespace point_layout

// sensor_msgs/PointCloud2 with fields x, y, z, intensity, sec, nsec.
template <typename PointCloud2Msg>
void toPointCloud2Msg(const rsf::PointCloud& cloud, const std::string& frame_id,
                      PointCloud2Msg& msg) {
  fillHeader(cloud.stamp, frame_id, msg.header);

  msg.height = 1;
  msg.width = static_cast<std::uint32_t>(cloud.points.size());
  msg.is_dense = true;
  msg.is_bigendian = false;
  msg.point_step = point_layout::kPointStep;
  msg.row_step = msg.point_step * msg.width;

  msg.fields.resize(6);
  const char* const names[6] = {"x", "y", "z", "intensity", "sec", "nsec"};
  const std::uint32_t offsets[6] = {point_layout::kOffsetX,         point_layout::kOffsetY,
                                    point_layout::kOffsetZ,         point_layout::kOffsetIntensity,
                                    point_layout::kOffsetSec,       point_layout::kOffsetNsec};
  const std::uint8_t datatypes[6] = {point_field::kFloat32, point_field::kFloat32,
                                     point_field::kFloat32, point_field::kFloat32,
                                     point_field::kUint32,  point_field::kUint32};
  for (std::size_t i = 0; i < 6; ++i) {
    msg.fields[i].name = names[i];
    msg.fields[i].offset = offsets[i];
    msg.fields[i].datatype = datatypes[i];
    msg.fields[i].count = 1;
  }

  // Written field by field in little-endian order, matching is_bigendian above.
  // This avoids depending on the memory layout of rsf::PointXYZIT.
  msg.data.resize(static_cast<std::size_t>(msg.row_step));
  std::uint8_t* cursor = msg.data.data();
  for (const rsf::PointXYZIT& point : cloud.points) {
    rsf::writeF32(cursor + point_layout::kOffsetX, point.x);
    rsf::writeF32(cursor + point_layout::kOffsetY, point.y);
    rsf::writeF32(cursor + point_layout::kOffsetZ, point.z);
    rsf::writeF32(cursor + point_layout::kOffsetIntensity, point.intensity);
    rsf::writeU32(cursor + point_layout::kOffsetSec, point.sec);
    rsf::writeU32(cursor + point_layout::kOffsetNsec, point.nsec);
    cursor += point_layout::kPointStep;
  }
}

// ******************** IMU ********************

// sensor_msgs/Imu. The measurements are in the IMU frame, which is rotated with
// respect to the LiDAR frame (section 3.8.7); publishing them under their own
// frame_id leaves that rotation to the TF tree rather than baking it in here.
// Orientation and all covariances are reported as transmitted, and the
// specification marks them unsupported.
template <typename ImuMsg>
void toImuMsg(const rsf::Imu& imu, const std::string& frame_id, ImuMsg& msg) {
  fillHeader(imu.stamp, frame_id, msg.header);

  msg.orientation.x = imu.orientation.x;
  msg.orientation.y = imu.orientation.y;
  msg.orientation.z = imu.orientation.z;
  msg.orientation.w = imu.orientation.w;

  msg.angular_velocity.x = imu.angular_velocity.x;
  msg.angular_velocity.y = imu.angular_velocity.y;
  msg.angular_velocity.z = imu.angular_velocity.z;

  msg.linear_acceleration.x = imu.linear_acceleration.x;
  msg.linear_acceleration.y = imu.linear_acceleration.y;
  msg.linear_acceleration.z = imu.linear_acceleration.z;

  for (std::size_t i = 0; i < 9; ++i) {
    msg.orientation_covariance[i] = imu.orientation_covariance[i];
    msg.angular_velocity_covariance[i] = imu.angular_velocity_covariance[i];
    msg.linear_acceleration_covariance[i] = imu.linear_acceleration_covariance[i];
  }
}

// ******************** GNSS ********************

// sensor_msgs/NavSatFix.
template <typename NavSatFixMsg>
void toNavSatFixMsg(const rsf::NavSatFix& fix, const std::string& frame_id, NavSatFixMsg& msg) {
  fillHeader(fix.stamp, frame_id, msg.header);

  msg.latitude = fix.latitude;
  msg.longitude = fix.longitude;
  msg.altitude = fix.altitude;
  for (std::size_t i = 0; i < 9; ++i) {
    msg.position_covariance[i] = fix.position_covariance[i];
  }

  msg.status.status = fix.status;
  msg.status.service = fix.service;
  msg.position_covariance_type = fix.position_covariance_type;
}

// nmea_msgs/Gpgga. Latitude and longitude stay in the transmitted ddmm.mmmm
// form, as the message definition expects.
template <typename GpggaMsg>
void toGpggaMsg(const rsf::GgaSentence& gga, const std::string& frame_id, GpggaMsg& msg) {
  fillHeader(gga.stamp, frame_id, msg.header);

  msg.message_id = gga.message_id;
  msg.utc_seconds = gga.utc_second;
  msg.lat = gga.latitude;
  msg.lon = gga.longitude;
  msg.lat_dir = charToString(gga.latitude_direction);
  msg.lon_dir = charToString(gga.longitude_direction);
  msg.gps_qual = gga.position_status;
  msg.num_sats = gga.num_satellites;
  msg.hdop = gga.hdop;
  msg.alt = gga.altitude;
  msg.altitude_units = charToString(gga.altitude_units);
  msg.undulation = gga.undulation;
  msg.undulation_units = charToString(gga.undulation_units);
  msg.diff_age = gga.diff_age;
  msg.station_id = gga.station_id;
}

// nmea_msgs/Gprmc. Section 3.8.5 transmits latitude and longitude in degrees
// rather than the ddmm.mmmm used by GGA, so they are copied through unchanged.
template <typename GprmcMsg>
void toGprmcMsg(const rsf::RmcSentence& rmc, const std::string& frame_id, GprmcMsg& msg) {
  fillHeader(rmc.stamp, frame_id, msg.header);

  msg.message_id = rmc.message_id;
  msg.utc_seconds = rmc.utc_second;
  msg.position_status = charToString(rmc.status);
  msg.lat = rmc.latitude;
  msg.lon = rmc.longitude;
  msg.lat_dir = charToString(rmc.latitude_direction);
  msg.lon_dir = charToString(rmc.longitude_direction);
  msg.speed = static_cast<float>(rmc.speed);
  msg.track = static_cast<float>(rmc.track);
  msg.date = rmc.date;
  msg.mag_var = static_cast<float>(rmc.magnetic_variation);
  msg.mag_var_direction = charToString(rmc.magnetic_variation_direction);
  msg.mode_indicator = charToString(rmc.mode);
}

// nmea_msgs/Gpzda.
template <typename GpzdaMsg>
void toGpzdaMsg(const rsf::ZdaSentence& zda, const std::string& frame_id, GpzdaMsg& msg) {
  fillHeader(zda.stamp, frame_id, msg.header);

  msg.message_id = zda.message_id;
  msg.utc_seconds = zda.utc_second;
  msg.day = zda.day;
  msg.month = zda.month;
  msg.year = zda.year;
  msg.hour_offset_gmt = zda.hour_offset_gmt;
  msg.minute_offset_gmt = zda.minute_offset_gmt;
}

// ******************** Diagnostics ********************

// diagnostic_msgs/DiagnosticArray with a single status entry.
//
// The numeric key/value pairs are kept in the raw form the sensor sends, so
// that anything parsing them keeps working, and the decoded names are added
// alongside under separate keys for the benefit of rqt_runtime_monitor.
template <typename DiagnosticArrayMsg>
void toDiagnosticArrayMsg(const rsf::Diagnostics& diagnostics, const std::string& status_name,
                          DiagnosticArrayMsg& msg) {
  fillHeader(diagnostics.stamp, std::string(), msg.header);

  msg.status.resize(1);
  auto& status = msg.status[0];
  status.name = status_name;
  status.hardware_id = diagnostics.device_id;
  status.level = diagnostics.healthy() ? diagnostic_level::kOk : diagnostic_level::kWarn;
  status.message = diagnostics.healthy() ? "OK" : "device reports a fault";

  status.values.clear();
  const auto add = [&status](const std::string& key, const std::string& value) {
    auto& entry = status.values.emplace_back();
    entry.key = key;
    entry.value = value;
  };

  add("ip_address", diagnostics.ip_address);
  add("ip_port", std::to_string(diagnostics.port));
  add("product_name", diagnostics.product_name);
  add("firmware_version", diagnostics.softwareVersion());
  add("device_id", diagnostics.device_id);
  add("device_status", std::to_string(static_cast<int>(diagnostics.device_status)));
  add("device_temperature", std::to_string(diagnostics.device_temperature));
  add("cpu_usage", std::to_string(static_cast<int>(diagnostics.cpu_usage)));
  add("elapsed_time", std::to_string(diagnostics.elapsed_time));

  add("odometry_state", std::to_string(static_cast<int>(diagnostics.odometry_state)));
  add("odometry_type", std::to_string(static_cast<int>(diagnostics.odometry_type)));
  add("gnss_state", std::to_string(static_cast<int>(diagnostics.gnss_state)));
  add("gnss_type", std::to_string(static_cast<int>(diagnostics.gnss_type)));

  add("odometry_state_text", rsf::toString(diagnostics.odometryState()));
  add("odometry_source", rsf::toString(diagnostics.odometrySource().kind));
  add("gnss_state_text", rsf::toString(diagnostics.gnssState()));
  add("gnss_source", rsf::toString(diagnostics.gnssSource().kind));
}

// ******************** Text ********************

// std_msgs/String, used for the state and type strings of sections 3.8.8/3.8.9.
template <typename StringMsg>
void toStringMsg(const std::string& text, StringMsg& msg) {
  msg.data = text;
}

}  // namespace rsf_ros
