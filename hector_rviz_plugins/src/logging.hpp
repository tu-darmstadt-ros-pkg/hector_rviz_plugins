//
// Created by stefan on 16.12.24.
//

#ifndef HECTOR_RVIZ_PLUGINS_LOGGING_HPP
#define HECTOR_RVIZ_PLUGINS_LOGGING_HPP

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

namespace hector_rviz_plugins
{
inline rclcpp::Clock &get_logging_clock()
{
  static rclcpp::Clock clock;
  return clock;
}
} // namespace hector_rviz_plugins

#define HECTOR_RVIZ_LOG_DEBUG( ... )                                                               \
  RCLCPP_DEBUG( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_DEBUG_STREAM( ... )                                                        \
  RCLCPP_DEBUG_STREAM( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_DEBUG_ONCE( ... )                                                          \
  RCLCPP_DEBUG_ONCE( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_DEBUG_THROTTLE( duration, ... )                                            \
  RCLCPP_DEBUG_THROTTLE( rclcpp::get_logger( "hector_rviz_plugins" ),                              \
                         hector_rviz_plugins::get_logging_clock(), duration, __VA_ARGS__ )

#define HECTOR_RVIZ_LOG_INFO( ... )                                                                \
  RCLCPP_INFO( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_INFO_STREAM( ... )                                                         \
  RCLCPP_INFO_STREAM( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_INFO_ONCE( ... )                                                           \
  RCLCPP_INFO_ONCE( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_INFO_THROTTLE( duration, ... )                                             \
  RCLCPP_INFO_THROTTLE( rclcpp::get_logger( "hector_rviz_plugins" ),                               \
                        hector_rviz_plugins::get_logging_clock(), duration, __VA_ARGS__ )

#define HECTOR_RVIZ_LOG_WARN( ... )                                                                \
  RCLCPP_WARN( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_WARN_STREAM( ... )                                                         \
  RCLCPP_WARN_STREAM( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_WARN_ONCE( ... )                                                           \
  RCLCPP_WARN_ONCE( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_WARN_THROTTLE( duration, ... )                                             \
  RCLCPP_WARN_THROTTLE( rclcpp::get_logger( "hector_rviz_plugins" ),                               \
                        hector_rviz_plugins::get_logging_clock(), duration, __VA_ARGS__ )

#define HECTOR_RVIZ_LOG_ERROR( ... )                                                               \
  RCLCPP_ERROR( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_ERROR_STREAM( ... )                                                        \
  RCLCPP_ERROR_STREAM( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_ERROR_ONCE( ... )                                                          \
  RCLCPP_ERROR_ONCE( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_ERROR_THROTTLE( duration, ... )                                            \
  RCLCPP_ERROR_THROTTLE( rclcpp::get_logger( "hector_rviz_plugins" ),                              \
                         hector_rviz_plugins::get_logging_clock(), duration, __VA_ARGS__ )

#define HECTOR_RVIZ_LOG_FATAL( ... )                                                               \
  RCLCPP_FATAL( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_FATAL_STREAM( ... )                                                        \
  RCLCPP_FATAL_STREAM( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_FATAL_ONCE( ... )                                                          \
  RCLCPP_FATAL_ONCE( rclcpp::get_logger( "hector_rviz_plugins" ), __VA_ARGS__ )
#define HECTOR_RVIZ_LOG_FATAL_THROTTLE( duration, ... )                                            \
  RCLCPP_FATAL_THROTTLE( rclcpp::get_logger( "hector_rviz_plugins" ),                              \
                         hector_rviz_plugins::get_logging_clock(), duration, __VA_ARGS__ )

#endif // HECTOR_RVIZ_PLUGINS_LOGGING_HPP
