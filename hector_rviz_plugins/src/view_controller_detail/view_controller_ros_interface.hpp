//
// Created by stefan on 17.12.24.
//

#ifndef HECTOR_RVIZ_PLUGINS_VIEW_CONTROLLER_ROS_INTERFACE_HPP
#define HECTOR_RVIZ_PLUGINS_VIEW_CONTROLLER_ROS_INTERFACE_HPP

#include <hector_rviz_plugins_msgs/srv/move_eye.hpp>
#include <hector_rviz_plugins_msgs/srv/move_eye_and_focus.hpp>
#include <hector_rviz_plugins_msgs/srv/set_view_mode.hpp>
#include <hector_rviz_plugins_msgs/srv/track_frame.hpp>
#include <hector_rviz_plugins_msgs/msg/relative_view_controller_cmd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace hector_rviz_plugins
{

class ViewControllerRosInterface
{
public:
  ViewControllerRosInterface( HectorViewController &view_controller, rclcpp::Node::SharedPtr node,
                              rviz_common::FrameManagerIface *frame_manager )
      : view_controller_( view_controller ), frame_manager_( frame_manager ),
        node_( std::move( node ) )
  {
  }

  void disableTopics()
  {
    tracked_frame_pub_.reset();
    view_mode_pub_.reset();
  }

  void enableTopics()
  {
    if ( tracked_frame_pub_ == nullptr ) {
      tracked_frame_pub_ = node_->create_publisher<std_msgs::msg::String>(
          "~/hector_view_controller/tracked_frame", rclcpp::QoS( 1 ).transient_local() );
    }
    if ( view_mode_pub_ == nullptr ) {
      view_mode_pub_ = node_->create_publisher<hector_rviz_plugins_msgs::msg::ViewMode>(
          "~/hector_view_controller/view_mode", rclcpp::QoS( 1 ).transient_local() );
    }
    publishTrackedFrame();
    publishViewMode();
    relative_cmd_sub_ = node_->create_subscription<hector_rviz_plugins_msgs::msg::RelativeViewControllerCmd>(
        "~/hector_view_controller/relative_cmds", rclcpp::QoS( 1 ).transient_local(),
        [this]( const hector_rviz_plugins_msgs::msg::RelativeViewControllerCmd::ConstSharedPtr &msg ) {
          Ogre::Vector3 translation;
          translation.x = msg->translation.x;
          translation.y = msg->translation.y;
          translation.z = msg->translation.z;
          view_controller_.relativeViewControllerCmd( msg->yaw_delta, msg->theta_delta, msg->zoom_factor, translation, msg->stop_tracking,
                                     !msg->disable_animation, msg->switch_to_3d_mode );
        } );
  }

  void disableServices()
  {
    move_eye_and_focus_service_.reset();
    move_eye_service_.reset();
    set_view_mode_service_.reset();
    track_frame_service_.reset();
  }

  void enableServices()
  {
    using namespace hector_rviz_plugins_msgs::srv;
    if ( move_eye_service_ == nullptr ) {
      move_eye_service_ = node_->create_service<MoveEye>(
          "~/hector_view_controller/move_eye",
          [this]( const MoveEye::Request::SharedPtr &req, const MoveEye::Response::SharedPtr & ) {
            if ( req->header.frame_id.empty() )
              req->header.frame_id = frame_manager_->getFixedFrame();
            geometry_msgs::msg::Pose pose;
            pose.orientation.w = 1;
            pose.position = req->eye;
            Ogre::Vector3 pos;
            Ogre::Quaternion _;
            if ( !frame_manager_->transform( req->header, pose, pos, _ ) ) {
              HECTOR_RVIZ_LOG_WARN( "Failed to transform MoveEye target point to fixed frame." );
              return;
            }
            view_controller_.moveEyeWithFocusTo( pos, req->stop_tracking, !req->disable_animation,
                                                 req->switch_to_3d_mode );
          } );
    }
    if ( move_eye_and_focus_service_ == nullptr ) {
      move_eye_and_focus_service_ = node_->create_service<MoveEyeAndFocus>(
          "~/hector_view_controller/move_eye_and_focus",
          [this]( const MoveEyeAndFocus::Request::SharedPtr &req,
                  const MoveEyeAndFocus::Response::SharedPtr & ) {
            if ( req->header.frame_id.empty() )
              req->header.frame_id = frame_manager_->getFixedFrame();
            geometry_msgs::msg::Pose pose;
            pose.orientation.w = 1;
            pose.position = req->eye;
            Ogre::Vector3 eye;
            Ogre::Vector3 focus;
            Ogre::Quaternion _;
            if ( !frame_manager_->transform( req->header, pose, eye, _ ) ) {
              HECTOR_RVIZ_LOG_WARN(
                  "Failed to transform MoveEyeAndFocus eye point to fixed frame." );
              return;
            }
            pose.position = req->focus;
            if ( !frame_manager_->transform( req->header, pose, focus, _ ) ) {
              HECTOR_RVIZ_LOG_WARN(
                  "Failed to transform MoveEyeAndFocus focus point to fixed frame." );
              return;
            }
            view_controller_.moveEyeWithNewFocus( eye, focus, req->stop_tracking,
                                                  !req->disable_animation );
            return;
          } );
    }
    if ( set_view_mode_service_ == nullptr ) {
      set_view_mode_service_ = node_->create_service<SetViewMode>(
          "~/hector_view_controller/set_view_mode",
          [this]( const SetViewMode::Request::SharedPtr &req,
                  const SetViewMode::Response::SharedPtr & ) {
            view_controller_.setMode( req->mode.mode == hector_rviz_plugins_msgs::msg::ViewMode::MODE_3D
                                          ? view_modes::Mode3D
                                          : view_modes::Mode2D,
                                      !req->disable_animation );
            return true;
          } );
    }
    if ( track_frame_service_ == nullptr ) {
      track_frame_service_ =
          node_->create_service<TrackFrame>( "~/hector_view_controller/set_tracked_frame",
                                             [this]( const TrackFrame::Request::SharedPtr &req,
                                                     const TrackFrame::Response::SharedPtr & ) {
                                               view_controller_.trackFrame( req->frame );
                                               return true;
                                             } );
    }
  }

  void publishViewMode()
  {
    if ( view_mode_pub_ == nullptr )
      return;
    hector_rviz_plugins_msgs::msg::ViewMode view_mode_msg;
    view_mode_msg.mode = view_controller_.mode() == view_modes::Mode2D
                             ? hector_rviz_plugins_msgs::msg::ViewMode::MODE_2D
                             : hector_rviz_plugins_msgs::msg::ViewMode::MODE_3D;
    view_mode_pub_->publish( view_mode_msg );
  }

  void publishTrackedFrame()
  {
    if ( tracked_frame_pub_ == nullptr )
      return;
    std_msgs::msg::String tracked_frame_msg;
    tracked_frame_msg.data =
        view_controller_.isTrackingFrame() ? view_controller_.trackedFrame() : "";
    tracked_frame_pub_->publish( tracked_frame_msg );
  }

private:
  HectorViewController &view_controller_;
  rviz_common::FrameManagerIface *frame_manager_;

  /* ROS Interface */
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracked_frame_pub_;
  rclcpp::Publisher<hector_rviz_plugins_msgs::msg::ViewMode>::SharedPtr view_mode_pub_;
  rclcpp::Subscription<hector_rviz_plugins_msgs::msg::RelativeViewControllerCmd>::SharedPtr relative_cmd_sub_;
  rclcpp::Service<hector_rviz_plugins_msgs::srv::MoveEye>::SharedPtr move_eye_service_;
  rclcpp::Service<hector_rviz_plugins_msgs::srv::MoveEyeAndFocus>::SharedPtr move_eye_and_focus_service_;
  rclcpp::Service<hector_rviz_plugins_msgs::srv::TrackFrame>::SharedPtr track_frame_service_;
  rclcpp::Service<hector_rviz_plugins_msgs::srv::SetViewMode>::SharedPtr set_view_mode_service_;
};
} // namespace hector_rviz_plugins

#endif // HECTOR_RVIZ_PLUGINS_VIEW_CONTROLLER_ROS_INTERFACE_HPP
