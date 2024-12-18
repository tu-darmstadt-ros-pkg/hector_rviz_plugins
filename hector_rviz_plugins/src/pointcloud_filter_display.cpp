/*
 * Copyright (C) 2020  Jasper Suess, Stefan Fabian
 *
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public Licen+se for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "hector_rviz_plugins/pointcloud_filter_display.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/transformation/transformation_manager.hpp>
#include <rviz_common/validate_floats.hpp>

#include <rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp>

namespace hector_rviz_plugins
{
PointCloudFilterDisplay::PointCloudFilterDisplay()
    : cloud_queue_(),
      point_cloud_common_( std::make_unique<rviz_default_plugins::PointCloudCommon>( this ) )
{
  using namespace rviz_common::properties;

  filter_property_ = new BoolProperty( "Enable filtering", true, "Enable/Disable all filters", this,
                                       SLOT( updateParameters() ), this );

  radial_filter_property_ =
      new BoolProperty( "Radial Filter", false, "Eable/Disable radial filtering", this,
                        SLOT( updateParameters() ), this );
  max_radial_distance_property_ = new FloatProperty(
      "Radius", 5.0, "Maximum distance from the origin for points to be displayed in meter.", this,
      SLOT( updateParameters() ), this );

  x_filter_property_ = new BoolProperty( "X Filter", false, "Activates X-Coord Filter", this,
                                         SLOT( updateParameters() ), this );
  x_min_value_property_ =
      new FloatProperty( "X Min Value", -2.0, "Minimum value for points to be displayed.", this,
                         SLOT( updateParameters() ), this );
  x_max_value_property_ =
      new FloatProperty( "X Max Value", 2.0, "Maximum value for points to be displayed.", this,
                         SLOT( updateParameters() ), this );

  y_filter_property_ = new BoolProperty( "Y Filter", false, "Activates Y-Coord Filter", this,
                                         SLOT( updateParameters() ), this );
  y_min_value_property_ =
      new FloatProperty( "Y Min Value", -2.0, "Minimum value for points to be displayed.", this,
                         SLOT( updateParameters() ), this );
  y_max_value_property_ =
      new FloatProperty( "Y Max Value", 2.0, "Maximum value for points to be displayed.", this,
                         SLOT( updateParameters() ), this );

  z_filter_property_ = new BoolProperty( "Z Filter", false, "Activates Z-Coord Filter", this,
                                         SLOT( updateParameters() ), this );
  z_min_value_property_ =
      new FloatProperty( "Z Min Value", -2.0, "Minimum value for points to be displayed.", this,
                         SLOT( updateParameters() ), this );
  z_max_value_property_ =
      new FloatProperty( "Z Max Value", 2.0, "Maximum value for points to be displayed.", this,
                         SLOT( updateParameters() ), this );
  filter_by_channel_value_property_ =
      new BoolProperty( "Filter by Channel Value", false, "Enable/Disable filter by channel value",
                        this, SLOT( updateParameters() ), this );
  channel_property_ = new EditableEnumProperty( "Channel", "", "The channel to filter by", this,
                                                SLOT( updateParameters() ), this );
  channel_min_value_property_ =
      new FloatProperty( "Channel Min Value", -255.0, "Minimum value for points to be displayed.",
                         this, SLOT( updateParameters() ), this );
  channel_max_value_property_ =
      new FloatProperty( "Channel Max Value", 255.0, "Maximum value for points to be displayed.",
                         this, SLOT( updateParameters() ), this );

  frame_property_ = new TfFrameProperty(
      "Frame", "<Fixed Frame>", "The frame to which the points are filtered to relatively.", this,
      nullptr, true, SLOT( updateParameters() ), this );

  use_axes_frame_property_ =
      new BoolProperty( "Use other Axes", false,
                        "Whether to use a different Frame for the axes that filtering is based on",
                        this, SLOT( updateParameters() ), this );
  axes_frame_property_ =
      new TfFrameProperty( "Axes Frame", "<Fixed Frame>", "The frame used for the filter axes.",
                           this, nullptr, true, SLOT( updateParameters() ), this );
}

PointCloudFilterDisplay::~PointCloudFilterDisplay() = default;

void PointCloudFilterDisplay::onInitialize()
{
  MFDClass::onInitialize();

  frame_property_->setFrameManager( context_->getFrameManager() );
  axes_frame_property_->setFrameManager( context_->getFrameManager() );
  point_cloud_common_->initialize( context_, scene_node_ );
  updateParameters();
}

bool PointCloudFilterDisplay::isFilterActive() const
{
  return filter_property_->getBool() &&
         ( radial_filter_property_->getBool() || x_filter_property_->getBool() ||
           y_filter_property_->getBool() || z_filter_property_->getBool() ||
           filter_by_channel_value_property_->getBool() );
}

void PointCloudFilterDisplay::processMessage( sensor_msgs::msg::PointCloud2::ConstSharedPtr msg )
{
  // Check for old clouds outside of decay time
  auto now = context_->getClock()->now();
  while ( !cloud_queue_.empty() && ( now - cloud_queue_.front()->header.stamp ).seconds() >
                                       point_cloud_common_->decay_time_property_->getFloat() ) {
    cloud_queue_.pop_front();
  }

  cloud_queue_.emplace_back( msg );
  channel_property_->clearOptions();
  for ( const auto &field : msg->fields ) { channel_property_->addOptionStd( field.name ); }

  if ( isFilterActive() ) {
    sensor_msgs::msg::PointCloud2::SharedPtr filtered = filterPointCloud( msg );
    if ( filtered ) {
      point_cloud_common_->addMessage( filtered );
    }
  } else {
    point_cloud_common_->addMessage( msg );
  }
}

bool PointCloudFilterDisplay::getTransform( const std::string &source_frame,
                                            const tf2::TimePoint &time_point,
                                            Ogre::Vector3 &translation, Ogre::Quaternion &orientation )
{
  using rviz_common::properties::StatusProperty;
  std::string filter_frame = frame_property_->getFrameStd();
  if ( filter_frame == "<Fixed Frame>" )
    filter_frame = context_->getFixedFrame().toStdString();
  std::string axes_frame =
      use_axes_frame_property_->getBool() ? axes_frame_property_->getStdString() : filter_frame;
  if ( axes_frame == "<Fixed Frame>" )
    axes_frame = context_->getFixedFrame().toStdString();
  const auto transformer = context_->getTransformationManager()->getCurrentTransformer();
  std::string error;
  if ( !transformer->canTransform( filter_frame, source_frame, time_point, &error ) ) {
    setStatusStd( StatusProperty::Error, "Transform",
                  "Filter frame could not be transformed: " + error );
    return false;
  }
  geometry_msgs::msg::TransformStamped transform =
      transformer->lookupTransform( filter_frame, source_frame, time_point );
  translation.x = static_cast<float>( transform.transform.translation.x );
  translation.y = static_cast<float>( transform.transform.translation.y );
  translation.z = static_cast<float>( transform.transform.translation.z );
  if ( axes_frame != filter_frame ) {
    if ( !transformer->canTransform( axes_frame, source_frame, time_point, &error ) ) {
      setStatusStd( StatusProperty::Error, "Transform",
                    "Axes frame could not be transformed: " + error );
      return false;
    }
    transform = transformer->lookupTransform( axes_frame, source_frame, time_point );
  }
  orientation.w = static_cast<float>( transform.transform.rotation.w );
  orientation.x = static_cast<float>( transform.transform.rotation.x );
  orientation.y = static_cast<float>( transform.transform.rotation.y );
  orientation.z = static_cast<float>( transform.transform.rotation.z );
  return true;
}

namespace
{

float getChannelValue( const unsigned char *data_ptr, const uint8_t datatype )
{
  switch ( datatype ) {
  case sensor_msgs::msg::PointField::FLOAT32:
    return *reinterpret_cast<const float *>( data_ptr );
  case sensor_msgs::msg::PointField::FLOAT64:
    return *reinterpret_cast<const double *>( data_ptr );
  case sensor_msgs::msg::PointField::INT8:
    return *reinterpret_cast<const int8_t *>( data_ptr );
  case sensor_msgs::msg::PointField::UINT8:
    return *reinterpret_cast<const uint8_t *>( data_ptr );
  case sensor_msgs::msg::PointField::INT16:
    return *reinterpret_cast<const int16_t *>( data_ptr );
  case sensor_msgs::msg::PointField::UINT16:
    return *reinterpret_cast<const uint16_t *>( data_ptr );
  case sensor_msgs::msg::PointField::INT32:
    return *reinterpret_cast<const int32_t *>( data_ptr );
  case sensor_msgs::msg::PointField::UINT32:
    return *reinterpret_cast<const uint32_t *>( data_ptr );
  default:
    return 0.0;
  }
}

bool inBounds( const float value, const float min, const float max )
{
  return min <= value && value <= max;
}
} // namespace

sensor_msgs::msg::PointCloud2::SharedPtr
PointCloudFilterDisplay::filterPointCloud( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg )
{
  using rviz_common::properties::StatusProperty;
  int32_t xi = rviz_default_plugins::findChannelIndex( msg, "x" );
  int32_t yi = rviz_default_plugins::findChannelIndex( msg, "y" );
  int32_t zi = rviz_default_plugins::findChannelIndex( msg, "z" );
  if ( xi == -1 || yi == -1 || zi == -1 ) {
    return nullptr;
  }
  const uint32_t x_off = msg->fields[xi].offset;
  const uint32_t y_off = msg->fields[yi].offset;
  const uint32_t z_off = msg->fields[zi].offset;
  const uint32_t point_step = msg->point_step;
  const size_t point_count = msg->width * msg->height;

  if ( point_count * point_step != msg->data.size() ) {
    std::stringstream ss;
    ss << "Data size (" << msg->data.size() << " bytes) does not match width (" << msg->width
       << ") times height (" << msg->height << ") times point_step (" << point_step
       << ").  Dropping message.";
    setStatusStd( StatusProperty::Error, "Message", ss.str() );
    return nullptr;
  }

  auto filtered_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  filtered_cloud->header = msg->header;
  filtered_cloud->fields = msg->fields;
  filtered_cloud->is_bigendian = msg->is_bigendian;
  filtered_cloud->point_step = msg->point_step;
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = false;
  filtered_cloud->data.reserve( msg->data.size() );

  Ogre::Vector3 translation;
  Ogre::Quaternion orientation;
  if ( !getTransform( msg->header.frame_id, tf2_ros::fromMsg( msg->header.stamp ), translation,
                      orientation ) ) {
    return nullptr;
  }

  // get active channel name
  bool use_channel_filter = filter_by_channel_value_property_->getBool();
  int32_t channel_i =
      rviz_default_plugins::findChannelIndex( msg, channel_property_->getStdString() );
  if ( use_channel_filter && channel_i == -1 ) {
    return nullptr;
  }
  const uint32_t channel_off = use_channel_filter ? msg->fields[channel_i].offset : 0;
  const uint8_t channel_type = use_channel_filter ? msg->fields[channel_i].datatype : 0;
  const float channel_min = channel_min_value_property_->getFloat();
  const float channel_max = channel_max_value_property_->getFloat();

  // Get the parameters for the filtering
  const bool use_radial_filter = radial_filter_property_->getBool();
  const float max_radial_dist_2 =
      max_radial_distance_property_->getFloat() * max_radial_distance_property_->getFloat();
  const bool use_x_filter = x_filter_property_->getBool();
  const bool use_y_filter = y_filter_property_->getBool();
  const bool use_z_filter = z_filter_property_->getBool();

  const unsigned char *input_data = msg->data.data();
  for ( size_t i = 0; i < point_count; ++i, input_data += point_step ) {
    const Ogre::Vector3 pt( *reinterpret_cast<const float *>( input_data + x_off ),
                            *reinterpret_cast<const float *>( input_data + y_off ),
                            *reinterpret_cast<const float *>( input_data + z_off ) );
    if ( use_channel_filter && !inBounds( getChannelValue( input_data + channel_off, channel_type ),
                                          channel_min, channel_max ) ) {
      continue;
    }
    if ( !rviz_common::validateFloats( pt.x ) || !rviz_common::validateFloats( pt.y ) ||
         !rviz_common::validateFloats( pt.z ) ) {
      continue;
    }
    const Ogre::Vector3 transformed_pt = orientation * pt + translation;
    if ( use_radial_filter && transformed_pt.squaredLength() > max_radial_dist_2 ) {
      continue;
    }
    if ( use_x_filter && !inBounds( transformed_pt.x, x_min_value_property_->getFloat(),
                                    x_max_value_property_->getFloat() ) ) {
      continue;
    }
    if ( use_y_filter && !inBounds( transformed_pt.y, y_min_value_property_->getFloat(),
                                    y_max_value_property_->getFloat() ) ) {
      continue;
    }
    if ( use_z_filter && !inBounds( transformed_pt.z, z_min_value_property_->getFloat(),
                                    z_max_value_property_->getFloat() ) ) {
      continue;
    }

    filtered_cloud->data.insert( filtered_cloud->data.end(), input_data, input_data + point_step );
  }
  filtered_cloud->width =
      static_cast<unsigned int>( filtered_cloud->data.size() / filtered_cloud->point_step );
  filtered_cloud->row_step = filtered_cloud->width * filtered_cloud->point_step;
  return filtered_cloud;
}

void PointCloudFilterDisplay::update( float wall_dt, float ros_dt )
{
  point_cloud_common_->update( wall_dt, ros_dt );
}

void PointCloudFilterDisplay::reset()
{
  MFDClass::reset();
  point_cloud_common_->reset();
  cloud_queue_.clear();
}

void PointCloudFilterDisplay::updateParameters()
{

  const bool filter = filter_property_->getBool();
  radial_filter_property_->setHidden( !filter );
  max_radial_distance_property_->setHidden( !filter || !radial_filter_property_->getBool() );
  x_filter_property_->setHidden( !filter );
  x_min_value_property_->setHidden( !filter || !x_filter_property_->getBool() );
  x_max_value_property_->setHidden( !filter || !x_filter_property_->getBool() );
  y_filter_property_->setHidden( !filter );
  y_min_value_property_->setHidden( !filter || !y_filter_property_->getBool() );
  y_max_value_property_->setHidden( !filter || !y_filter_property_->getBool() );
  z_filter_property_->setHidden( !filter );
  z_min_value_property_->setHidden( !filter || !z_filter_property_->getBool() );
  z_max_value_property_->setHidden( !filter || !z_filter_property_->getBool() );
  use_axes_frame_property_->setHidden( !filter );
  axes_frame_property_->setHidden( !filter || !use_axes_frame_property_->getBool() );
  filter_by_channel_value_property_->setHidden( !filter );
  channel_property_->setHidden( !filter || !filter_by_channel_value_property_->getBool() );
  channel_min_value_property_->setHidden( !filter || !filter_by_channel_value_property_->getBool() );
  channel_max_value_property_->setHidden( !filter || !filter_by_channel_value_property_->getBool() );

  // Process each saved cloud again with the changed parameters and pass to point_cloud_common
  point_cloud_common_->reset();
  for ( const auto &cloud : cloud_queue_ ) {
    if ( isFilterActive() ) {
      sensor_msgs::msg::PointCloud2::SharedPtr filtered = filterPointCloud( cloud );
      if ( filtered ) {
        point_cloud_common_->addMessage( filtered );
      }
    } else {
      point_cloud_common_->addMessage( cloud );
    }
  }
}

} // namespace hector_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( hector_rviz_plugins::PointCloudFilterDisplay, rviz_common::Display )
