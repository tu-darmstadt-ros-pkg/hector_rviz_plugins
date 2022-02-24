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

#include "hector_rviz_plugins/multi_robot_state_display.h"

#include <moveit/robot_state_rviz_plugin/robot_state_display.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/ros_topic_property.h>

namespace hector_rviz_plugins
{

class PrivateRobotStateDisplayHelper : public moveit_rviz_plugin::RobotStateDisplay
{
public:
  PrivateRobotStateDisplayHelper()
  {
    robot_state_topic_property_->blockSignals( true );
    robot_state_topic_property_->setValue(
        QString( "/rviz/multi_robot_state_display_junk_topic" ) );
    robot_state_topic_property_->setReadOnly( true );
    robot_state_topic_property_->hide();
    robot_state_topic_property_->blockSignals( false );
  }

  void forwardNewRobotState( const moveit_msgs::DisplayRobotState::ConstPtr &state )
  {
    newRobotStateCallback( state );
  }

  void setPositionAndOrientation( const Ogre::Vector3 &position, const Ogre::Quaternion &orientation )
  {
    if ( scene_node_ == nullptr )
      return;
    scene_node_->setPosition( position );
    scene_node_->setOrientation( orientation );
  }

protected:
  void onInitialize() override
  {
    moveit_rviz_plugin::RobotStateDisplay::onInitialize();
    robot_state_topic_property_->setValue(
        QString( "/rviz/multi_robot_state_display_junk_topic" ) );
    robot_state_topic_property_->setReadOnly( true );
    robot_state_topic_property_->hide();
  }
};

MultiRobotStateDisplay::MultiRobotStateDisplay()
{
  topic_property_ = new rviz::RosTopicProperty(
      "topic", "/display_multi_robot_state", "hector_rviz_plugins_msgs/DisplayMultiRobotState",
      "The to listen for hector_rviz_plugins_msgs/DisplayMultiRobotState message.", this,
      SLOT( onTopicChanged() ) );
  onTopicChanged();
}

MultiRobotStateDisplay::~MultiRobotStateDisplay() = default;

void MultiRobotStateDisplay::update( float wall_dt, float ros_dt )
{
  if ( last_message_ == nullptr )
    return;
  std::string default_frame = last_message_->header.frame_id;
  if ( default_frame.empty() )
    default_frame = fixed_frame_.toStdString();
  for ( auto &kvp : displays_ ) {
    if ( kvp.second->isEnabled() )
      kvp.second->update( wall_dt, ros_dt );
    const geometry_msgs::PoseStamped &pose = poses_.find( kvp.first )->second;
    std_msgs::Header header = pose.header;
    if ( header.frame_id.empty() )
      header.frame_id = default_frame;
    if ( header.stamp.isZero() )
      header.stamp = last_message_->header.stamp;
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    context_->getFrameManager()->transform( header, pose.pose, position, orientation );
    kvp.second->setPositionAndOrientation( position, orientation );
  }

  // State update is done after update method calls because in the first update the state is reset
  if ( needs_state_update_ ) {
    needs_state_update_ = false;
    for ( const auto &robot : last_message_->robots ) {
      if ( robot.id.empty() ) {
        ROS_ERROR_ONCE( "No id provided for robot state and the state was ignored! This message is "
                        "printed once!" );
        continue;
      }
      auto it = displays_.find( robot.id );
      if ( it->second->isEnabled() )
        it->second->forwardNewRobotState(
            boost::make_shared<const moveit_msgs::DisplayRobotState>( robot.robot_state ) );
    }
  }
}

void MultiRobotStateDisplay::onNewMultiRobotState(
    const hector_rviz_plugins_msgs::DisplayMultiRobotStateConstPtr &msg )
{
  last_message_ = msg;
  // Collect the robot ids to check if there are displays that can be removed
  std::set<std::string> keys;
  std::for_each( displays_.begin(), displays_.end(),
                 [&keys]( const std::pair<std::string, PrivateRobotStateDisplayHelper *> &s ) {
                   keys.insert( s.first );
                 } );
  for ( const auto &robot : msg->robots ) {
    if ( robot.id.empty() ) {
      ROS_ERROR_ONCE( "No id provided for robot state and the state was ignored! This message is "
                      "printed once!" );
      continue;
    }
    auto it = displays_.find( robot.id );
    if ( it == displays_.end() ) {
      it = displays_.emplace( robot.id, new PrivateRobotStateDisplayHelper() ).first;
      it->second->initialize( context_ );
      it->second->setName( QString::fromStdString( robot.id ) );
      it->second->setBool( getBool() );
      connect( it->second, &PrivateRobotStateDisplayHelper::changed, this,
               &MultiRobotStateDisplay::onSubdisplayEnableChanged );
      addChild( it->second );
      poses_.emplace( robot.id, robot.pose );
    }
    poses_.find( robot.id )->second = robot.pose;
    keys.erase( robot.id );
  }
  // Remove unused displays
  for ( const auto &key : keys ) {
    auto it = displays_.find( key );
    if ( it == displays_.end() )
      continue;
    takeChild( it->second );
    delete it->second;
    displays_.erase( it );
    poses_.erase( key );
  }
  needs_state_update_ = true;
}

void MultiRobotStateDisplay::onTopicChanged()
{
  sub_.shutdown();
  sub_ = update_nh_.subscribe<hector_rviz_plugins_msgs::DisplayMultiRobotState>(
      topic_property_->getTopicStd(), 10, &MultiRobotStateDisplay::onNewMultiRobotState, this );
}

void MultiRobotStateDisplay::onSubdisplayEnableChanged() { needs_state_update_ = true; }

void MultiRobotStateDisplay::onEnableChanged()
{
  for ( auto &kvp : displays_ ) { kvp.second->setBool( getBool() ); }
}
} // namespace hector_rviz_plugins

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( hector_rviz_plugins::MultiRobotStateDisplay, rviz::Display )
