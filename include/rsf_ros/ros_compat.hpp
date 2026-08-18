// rsf_ros - ROS glue shared by the RSF-X001 drivers.
//
// SPDX-License-Identifier: Apache-2.0
//
// The whole of the ROS 1 / ROS 2 difference that conversions.hpp has to care
// about, isolated in one place.
//
// This directory is deliberately named after neither ROS distribution: the
// ROS 2 package (rsf_node2) and a ROS 1 package would both include it under the
// same path and namespace.
//
// Message field names are identical between the two distributions, so the
// conversion code can be written once against a template parameter. The one
// exception is the timestamp:
//
//   ROS 2  builtin_interfaces::msg::Time { int32  sec; uint32 nanosec; }
//   ROS 1  ros::Time                     { uint32 sec; uint32 nsec;    }
//
// toRosTime() picks the right member at compile time, so neither node has to
// special-case it.

#pragma once

#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>

#include "rsf/time.hpp"

namespace rsf_ros {
namespace detail {

template <typename T, typename = void>
struct HasNanosecField : std::false_type {};

template <typename T>
struct HasNanosecField<T, decltype(void(std::declval<T&>().nanosec))> : std::true_type {};

}  // namespace detail

// Copies a sensor timestamp into a ROS time field of either distribution.
template <typename StampT>
void toRosTime(const rsf::Timestamp& time, StampT& stamp) {
  if constexpr (detail::HasNanosecField<StampT>::value) {
    stamp.sec = static_cast<decltype(stamp.sec)>(time.sec);
    stamp.nanosec = time.nsec;
  } else {
    stamp.sec = static_cast<decltype(stamp.sec)>(time.sec);
    stamp.nsec = time.nsec;
  }
}

// Fills the std_msgs/Header common to nearly every published message.
template <typename HeaderT>
void fillHeader(const rsf::Timestamp& time, const std::string& frame_id, HeaderT& header) {
  toRosTime(time, header.stamp);
  header.frame_id = frame_id;
}

// The protocol stores single characters (hemisphere, units, mode), while the
// nmea_msgs definitions declare them as strings. An unset character becomes an
// empty string rather than a string containing a NUL.
inline std::string charToString(char value) {
  return value == '\0' ? std::string() : std::string(1, value);
}

// Constants from sensor_msgs/PointField. Identical in ROS 1 and ROS 2, and
// repeated here so that conversions.hpp needs no message type for them.
namespace point_field {
constexpr std::uint8_t kUint32 = 6;
constexpr std::uint8_t kFloat32 = 7;
}  // namespace point_field

// Constants from diagnostic_msgs/DiagnosticStatus, likewise identical.
namespace diagnostic_level {
constexpr std::uint8_t kOk = 0;
constexpr std::uint8_t kWarn = 1;
constexpr std::uint8_t kError = 2;
constexpr std::uint8_t kStale = 3;
}  // namespace diagnostic_level

}  // namespace rsf_ros
