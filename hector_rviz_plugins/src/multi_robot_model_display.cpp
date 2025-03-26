/*
 * Copyright (C) 20245 Stefan Fabian
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

#include "hector_rviz_plugins/multi_robot_model_display.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_default_plugins/displays/robot_model/robot_model_display.hpp>

namespace hector_rviz_plugins
{
MultiRobotModelDisplay::MultiRobotModelDisplay()
{
  enable_scan_property_ = new rviz_common::properties::BoolProperty(
      "Enable Scan", true, "Enable scanning for new robot_description topics", this );
  scan_interval_property_ = new rviz_common::properties::FloatProperty(
      "Scan Interval", 5, "Interval in seconds between scans for new robot_description topics", this );
}

MultiRobotModelDisplay::~MultiRobotModelDisplay() = default;

void MultiRobotModelDisplay::onInitialize() { Display::onInitialize(); }

namespace
{
bool isRobotDescription( const std::string &name, const std::vector<std::string> &types )
{
  // robot_description is 17 characters long
  if ( name.length() < 17 || name.substr( name.length() - 17 ) != "robot_description" )
    return false;
  return !types.empty() &&
         std::find( types.begin(), types.end(), "std_msgs/msg/String" ) != types.end();
}
} // namespace

void MultiRobotModelDisplay::update( float wall_dt, float )
{
  scan_age_ += wall_dt;
  if ( !enable_scan_property_->getBool() )
    return;
  if ( scan_age_ < scan_interval_property_->getFloat() * 1E9 )
    return;
  scan_age_ = 0;

  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  auto topic_names_and_types = node->get_topic_names_and_types();
  for ( const auto &[name, types] : topic_names_and_types ) {
    if ( !isRobotDescription( name, types ) )
      continue;
    if ( node->get_publishers_info_by_topic( name ).empty() )
      continue; // No publishers
    if ( robot_model_displays_.find( name ) != robot_model_displays_.end() )
      continue; // already added
    auto robot_model_display = std::make_unique<rviz_default_plugins::displays::RobotModelDisplay>();
    addChild( robot_model_display.get() );
    robot_model_display->initialize( context_ );
    robot_model_display->setName( QString::fromStdString( name ) );
    robot_model_display->setTopic( QString::fromStdString( name ), "std_msgs/msg/String" );
    robot_model_display->setEnabled( true );
    robot_model_displays_.try_emplace( name, std::move( robot_model_display ) );
  }
}
} // namespace hector_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( hector_rviz_plugins::MultiRobotModelDisplay, rviz_common::Display )
