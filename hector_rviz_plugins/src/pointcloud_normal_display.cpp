/*
 * Copyright (C) 2025 Stefan Fabian
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

#include "hector_rviz_plugins/pointcloud_normal_display.hpp"
#include "hector_rviz_plugins/rendering/pointcloud_normal_visual.hpp"

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

namespace hector_rviz_plugins
{

PointCloudNormalDisplay::PointCloudNormalDisplay()
{
  color_property_ = new rviz_common::properties::ColorProperty(
      "Color", QColor( 100, 100, 255 ), "Color to draw the normals.", this, SLOT( updateColor() ) );
  normal_length_property_ = new rviz_common::properties::FloatProperty(
      "Normal Length", 0.05, "Length of the normal lines.", this, SLOT( updateNormalLength() ) );
}

PointCloudNormalDisplay::~PointCloudNormalDisplay() { }

void PointCloudNormalDisplay::onInitialize()
{
  visual_ = std::make_unique<PointCloudNormalVisual>( scene_node_ );
  updateColor();
  updateNormalLength();
  PC2RDClass::onInitialize();
}

namespace
{
bool hasField( const sensor_msgs::msg::PointCloud2 &msg, const std::string &field_name )
{
  for ( const auto &field : msg.fields ) {
    if ( field.name == field_name ) {
      return true;
    }
  }
  return false;
}
} // namespace

void PointCloudNormalDisplay::processMessage( sensor_msgs::msg::PointCloud2::ConstSharedPtr msg )
{
  if ( !hasField( *msg, "x" ) || !hasField( *msg, "y" ) || !hasField( *msg, "z" ) ) {
    setStatus( rviz_common::properties::StatusProperty::Error, "Message",
               "Missing x, y or z channel. Dropping message." );
    visual_->updateData( nullptr );
    return;
  }
  if ( !hasField( *msg, "normal_x" ) || !hasField( *msg, "normal_y" ) ||
       !hasField( *msg, "normal_z" ) ) {
    setStatus( rviz_common::properties::StatusProperty::Error, "Message",
               "Missing normal_x, normal_y or normal_z channel. Dropping message." );
    visual_->updateData( nullptr );
    return;
  }
  setStatus( rviz_common::properties::StatusProperty::Ok, "Message", "Message received." );

  visual_->updateData( msg );
  visual_->setVisible( true );
}
void PointCloudNormalDisplay::reset()
{
  PC2RDClass::reset();
  visual_->updateData( nullptr );
  visual_->setVisible( false );
}

void PointCloudNormalDisplay::update( float wall_dt, float ros_dt )
{
  PC2RDClass::update( wall_dt, ros_dt );
}

void PointCloudNormalDisplay::updateColor()
{
  visual_->setFlatColor( color_property_->getOgreColor() );
}

void PointCloudNormalDisplay::updateNormalLength()
{
  visual_->setNormalLength( normal_length_property_->getFloat() );
}

} // namespace hector_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( hector_rviz_plugins::PointCloudNormalDisplay, rviz_common::Display )
