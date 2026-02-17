#pragma once

#include <string>

// Starting from MoveIt2 for jazzy the API to convert the MoveItErrorCode to string has changed.
// Newer MoveIt2 provides:
//   - header: <moveit/utils/moveit_error_code.hpp>
//   - function: moveit::core::errorCodeToString(...)
// Older MoveIt2 provides:
//   - header: <moveit/utils/moveit_error_code.h>
//   - function: moveit::core::error_code_to_string(...)

#if __has_include(<moveit/utils/moveit_error_code.hpp>)
  #include <moveit/utils/moveit_error_code.hpp>
  #define EASY_MOTION_MOVEIT_HAS_ERROR_HPP 1
#else
  #include <moveit/utils/moveit_error_code.h>
  #define EASY_MOTION_MOVEIT_HAS_ERROR_HPP 0
#endif

#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace easy_motion
{

inline std::string moveitErrorToString(const moveit_msgs::msg::MoveItErrorCodes & msg)
{
  const moveit::core::MoveItErrorCode ec(msg);

#if EASY_MOTION_MOVEIT_HAS_ERROR_HPP
  return moveit::core::errorCodeToString(ec);
#else
  return moveit::core::error_code_to_string(ec);
#endif
}

inline std::string moveitErrorToString(const moveit::core::MoveItErrorCode & ec)
{
#if EASY_MOTION_MOVEIT_HAS_ERROR_HPP
  return moveit::core::errorCodeToString(ec);
#else
  return moveit::core::error_code_to_string(ec);
#endif
}

}  // namespace easy_motion
