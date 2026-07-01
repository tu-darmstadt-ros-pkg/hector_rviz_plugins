/*
 * Copyright (C) 2020  Stefan Fabian
 *
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "hector_rviz_plugins/multi_robot_state_display.hpp"

#include "./logging.hpp"

#include <moveit/rdf_loader/rdf_loader.hpp>
#include <moveit/robot_state_rviz_plugin/robot_state_display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <std_msgs/msg/string.hpp>

namespace hector_rviz_plugins
{

// A moveit RobotStateDisplay driven externally instead of through moveit's blocking loader: the
// URDF (and optional SRDF) are loaded asynchronously from std_msgs/String topics named by
// setRobotDescription(), and the robot state is supplied via forwardNewRobotState().
class PrivateRobotStateDisplayHelper : public moveit_rviz_plugin::RobotStateDisplay
{
public:
  PrivateRobotStateDisplayHelper() { disableRobotStateTopic(); }

  // Drives this robot's URDF source. While disabled this only stores the value; once enabled a real
  // change triggers moveit's reset() -> our onEnable() -> re-subscribe to the new description.
  void setRobotDescription( const std::string &name )
  {
    robot_description_property_->setStdString( name );
  }

  void forwardNewRobotState( const moveit_msgs::msg::DisplayRobotState::ConstSharedPtr &state )
  {
    last_state_ = state;
    newRobotStateCallback( state );
  }

  void setPositionAndOrientation( const Ogre::Vector3 &position, const Ogre::Quaternion &orientation )
  {
    if ( scene_node_ == nullptr )
      return;
    scene_node_->setPosition( position );
    scene_node_->setOrientation( orientation );
  }

  void setRobotAlpha( float alpha ) { robot_alpha_property_->setFloat( alpha ); }

  void setVisualVisible( bool visible ) { enable_visual_visible_->setBool( visible ); }

  void setCollisionVisible( bool visible ) { enable_collision_visible_->setBool( visible ); }

protected:
  void onInitialize() override
  {
    moveit_rviz_plugin::RobotStateDisplay::onInitialize();
    disableRobotStateTopic();
    // Visible so the user can see which URDF each robot uses, but not user-editable
    // (setStdString still drives the value from setRobotDescription()).
    robot_description_property_->setReadOnly( true );
  }

  // Skip moveit's blocking initializeLoader() (and changedRobotStateTopic(), which would create the
  // unwanted robot-state subscription). Call the grandparent Display::onEnable() for visibility
  // only, then subscribe to the robot description ourselves.
  void onEnable() override
  {
    rviz_common::Display::onEnable();
    subscribeRobotDescription();
    calculateOffsetPosition();
  }

  void onDisable() override
  {
    urdf_sub_.reset();
    srdf_sub_.reset();
    RobotStateDisplay::onDisable();
  }

private:
  // The robot states are forwarded manually via forwardNewRobotState(), so the inner display must
  // not subscribe on its own. Point it at a junk topic and hide the (read-only) property.
  void disableRobotStateTopic()
  {
    robot_state_topic_property_->blockSignals( true );
    robot_state_topic_property_->setValue(
        QString( "/rviz/multi_robot_state_display_junk_topic" ) );
    robot_state_topic_property_->setReadOnly( true );
    robot_state_topic_property_->hide();
    robot_state_topic_property_->blockSignals( false );
  }

  void subscribeRobotDescription()
  {
    urdf_sub_.reset();
    srdf_sub_.reset();
    urdf_string_.clear();
    srdf_string_.clear();
    const std::string name = robot_description_property_->getStdString();
    if ( name.empty() ) {
      setStatus( rviz_common::properties::StatusProperty::Error, "RobotModel",
                 "`Robot Description` is empty" );
      return;
    }
    const auto qos = rclcpp::QoS( 1 ).transient_local().reliable();
    urdf_sub_ = node_->create_subscription<std_msgs::msg::String>(
        name, qos, [this]( std_msgs::msg::String::ConstSharedPtr m ) { onUrdf( m->data ); } );
    srdf_sub_ = node_->create_subscription<std_msgs::msg::String>(
        name + "_semantic", qos,
        [this]( std_msgs::msg::String::ConstSharedPtr m ) { onSrdf( m->data ); } );
    setStatus( rviz_common::properties::StatusProperty::Warn, "RobotModel",
               "Waiting for robot description..." );
  }

  void onUrdf( const std::string &urdf )
  {
    urdf_string_ = urdf;
    rebuildModel();
  }

  void onSrdf( const std::string &srdf )
  {
    srdf_string_ = srdf;
    if ( !urdf_string_.empty() )
      rebuildModel();
  }

  void rebuildModel()
  {
    // RDFLoader fails to load the URDF if the SRDF string is empty, so substitute a minimal valid
    // SRDF for robots that publish no _semantic topic.
    const std::string srdf = srdf_string_.empty() ? std::string( "<robot/>" ) : srdf_string_;
    rdf_loader_ = std::make_shared<rdf_loader::RDFLoader>( urdf_string_, srdf );
    loadRobotModel();
    // loadRobotModel() resets the alpha; reapply it.
    if ( robot_ )
      robot_->setAlpha( robot_alpha_property_->getFloat() );
    if ( last_state_ )
      newRobotStateCallback( last_state_ ); // re-apply the last state on (re)build
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_, srdf_sub_;
  std::string urdf_string_, srdf_string_; // latest URDF/SRDF strings, empty if none yet
  moveit_msgs::msg::DisplayRobotState::ConstSharedPtr last_state_; // last forwarded state
};

MultiRobotStateDisplay::MultiRobotStateDisplay()
{
  // The topic and its QoS sub-properties (reliability, durability, depth, ...) are provided by the
  // RosTopicDisplay base.
  default_robot_description_property_ = new rviz_common::properties::StringProperty(
      "Robot Description",
      "robot_description", "Default robot description (topic name) used for robot entries that do not specify their own.",
      this, SLOT( onDefaultRobotDescriptionChanged() ) );
}

MultiRobotStateDisplay::~MultiRobotStateDisplay() = default;

void MultiRobotStateDisplay::update( float wall_dt, float ros_dt )
{
  if ( last_message_ == nullptr )
    return;
  std::string default_frame = last_message_->header.frame_id;
  if ( default_frame.empty() )
    default_frame = fixed_frame_.toStdString();

  int transform_failures = 0;
  for ( auto &[id, entry] : robots_ ) {
    if ( !entry.display->isEnabled() )
      continue;
    entry.display->update( wall_dt, ros_dt );

    std_msgs::msg::Header header = entry.pose.header;
    if ( header.frame_id.empty() )
      header.frame_id = default_frame;
    if ( header.stamp.sec == 0 && header.stamp.nanosec == 0 )
      header.stamp = last_message_->header.stamp;
    Ogre::Vector3 position( Ogre::Vector3::ZERO );
    Ogre::Quaternion orientation( Ogre::Quaternion::IDENTITY );
    if ( !context_->getFrameManager()->transform( header, entry.pose.pose, position, orientation ) ) {
      ++transform_failures;
      continue;
    }
    entry.display->setPositionAndOrientation( position, orientation );
  }
  if ( transform_failures > 0 ) {
    setStatus( rviz_common::properties::StatusProperty::Warn, "Transform",
               QString( "Could not transform the pose of %1 robot(s)." ).arg( transform_failures ) );
  } else {
    setStatus( rviz_common::properties::StatusProperty::Ok, "Transform", "OK" );
  }

  // State update is done after the update method calls because in the first update the state is reset.
  if ( needs_state_update_ ) {
    needs_state_update_ = false;
    for ( auto &[id, entry] : robots_ ) {
      if ( entry.display->isEnabled() && entry.state != nullptr )
        entry.display->forwardNewRobotState( entry.state );
    }
  }
}

void MultiRobotStateDisplay::processMessage(
    hector_rviz_plugins_msgs::msg::DisplayMultiRobotState::ConstSharedPtr msg )
{
  last_message_ = msg;
  // Collect the existing robot ids to find displays that are no longer present.
  std::set<std::string, std::less<>> stale_ids;
  for ( const auto &[id, entry] : robots_ ) stale_ids.insert( id );

  for ( const auto &robot : msg->robots ) {
    if ( robot.id.empty() ) {
      HECTOR_RVIZ_LOG_ERROR_ONCE( "No id provided for robot state and the state was ignored! "
                                  "This message is printed once!" );
      continue;
    }
    // Empty per-robot description falls back to the display-level default.
    const std::string resolved = robot.robot_description.empty()
                                     ? default_robot_description_property_->getStdString()
                                     : robot.robot_description;
    auto it = robots_.find( robot.id );
    if ( it == robots_.end() ) {
      it = robots_.try_emplace( robot.id ).first;
      it->second.display = std::make_unique<PrivateRobotStateDisplayHelper>();
      auto *display = it->second.display.get();
      display->initialize( context_ );
      display->setName( QString::fromStdString( robot.id ) );
      // Set the description while disabled (stores only); enabling below triggers onEnable() which
      // subscribes with the resolved description.
      display->setRobotDescription( resolved );
      connect( display, &PrivateRobotStateDisplayHelper::changed, this,
               &MultiRobotStateDisplay::onSubdisplayEnableChanged );
      addChild( display );
      display->setBool( getBool() );
    } else {
      it->second.display->setRobotDescription( resolved ); // re-subscribes only if changed
    }
    it->second.display->setRobotAlpha( robot.alpha );
    it->second.display->setVisualVisible( robot.show_visual );
    it->second.display->setCollisionVisible( robot.show_collision );
    it->second.pose = robot.pose;
    it->second.state =
        std::make_shared<const moveit_msgs::msg::DisplayRobotState>( robot.robot_state );
    it->second.robot_description = robot.robot_description;
    stale_ids.erase( robot.id );
  }

  // Remove displays for robots no longer present in the message.
  for ( const auto &id : stale_ids ) {
    auto it = robots_.find( id );
    if ( it == robots_.end() )
      continue;
    takeChild( it->second.display.get() );
    robots_.erase( it );
  }

  needs_state_update_ = true;
  setStatus( rviz_common::properties::StatusProperty::Ok, "Message",
             QString( "Showing %1 robot(s)." ).arg( robots_.size() ) );
}

void MultiRobotStateDisplay::reset()
{
  for ( auto &[id, entry] : robots_ ) takeChild( entry.display.get() );
  robots_.clear();
  last_message_.reset();
  needs_state_update_ = false;
  RTDClass::reset();
}

void MultiRobotStateDisplay::onSubdisplayEnableChanged() { needs_state_update_ = true; }

void MultiRobotStateDisplay::onDefaultRobotDescriptionChanged()
{
  const std::string def = default_robot_description_property_->getStdString();
  for ( auto &[id, entry] : robots_ )
    if ( entry.robot_description.empty() )
      entry.display->setRobotDescription( def ); // re-subscribes entries that follow the default
  // Replay state on the next update() so re-subscribed entries reapply it once reloaded.
  needs_state_update_ = true;
}
} // namespace hector_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( hector_rviz_plugins::MultiRobotStateDisplay, rviz_common::Display )
