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

#include "hector_rviz_plugins/rendering/pointcloud_normal_visual.hpp"

#include <OgreMaterialManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreVertexIndexData.h>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp>

namespace hector_rviz_plugins
{

PointCloudNormalVisual::PointCloudNormalVisual( Ogre::SceneNode *parent )
{
  scene_node_ = parent->createChildSceneNode();
  manual_object_ = std::make_unique<Ogre::ManualObject>( "pointcloud_normals" );
  manual_object_->setDynamic( true );
  manual_object_->setRenderingMinPixelSize( 0.001 );
  scene_node_->attachObject( manual_object_.get() );
  manual_object_->setVisible( true );
}

PointCloudNormalVisual::PointCloudNormalVisual() { parent_->removeAndDestroyChild( scene_node_ ); }

namespace
{
int32_t findFieldOffset( const sensor_msgs::msg::PointCloud2 &msg, const std::string &field_name )
{
  for ( const auto &field : msg.fields ) {
    if ( field.name == field_name ) {
      return static_cast<int32_t>( field.offset );
    }
  }
  return -1;
}
} // namespace

void PointCloudNormalVisual::updateData( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg )
{
  last_msg_ = msg;
  updateVisual();
}

void PointCloudNormalVisual::setFlatColor( const Ogre::ColourValue &color )
{
  color_ = color;
  updateVisual();
}

void PointCloudNormalVisual::setNormalLength( float length )
{
  normal_length_ = length;
  updateVisual();
}

void PointCloudNormalVisual::setVisible( bool value ) { manual_object_->setVisible( value ); }

void PointCloudNormalVisual::updateVisual()
{
  manual_object_->clear();
  if ( last_msg_ == nullptr )
    return;
  const int32_t x_offset = findFieldOffset( *last_msg_, "x" );
  const int32_t y_offset = findFieldOffset( *last_msg_, "y" );
  const int32_t z_offset = findFieldOffset( *last_msg_, "z" );
  const int32_t nx_offset = findFieldOffset( *last_msg_, "normal_x" );
  const int32_t ny_offset = findFieldOffset( *last_msg_, "normal_y" );
  const int32_t nz_offset = findFieldOffset( *last_msg_, "normal_z" );
  if ( x_offset == -1 || y_offset == -1 || z_offset == -1 || nx_offset == -1 || ny_offset == -1 ||
       nz_offset == -1 ) {
    return;
  }
  const size_t point_step = last_msg_->point_step;
  const size_t point_count = last_msg_->width * last_msg_->height;
  const unsigned char *data_ptr = last_msg_->data.data();
  manual_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
  manual_object_->estimateVertexCount( point_count * 2 );
  Ogre::AxisAlignedBox bounding_box;
  uint32_t index_count = 0;
  for ( size_t i = 0; i < point_count; ++i, data_ptr += point_step ) {
    const float x = *reinterpret_cast<const float *>( data_ptr + x_offset );
    const float y = *reinterpret_cast<const float *>( data_ptr + y_offset );
    const float z = *reinterpret_cast<const float *>( data_ptr + z_offset );
    const float nx = *reinterpret_cast<const float *>( data_ptr + nx_offset );
    const float ny = *reinterpret_cast<const float *>( data_ptr + ny_offset );
    const float nz = *reinterpret_cast<const float *>( data_ptr + nz_offset );

    const Ogre::Vector3 pt( x, y, z );
    if ( pt.isNaN() )
      continue;
    const Ogre::Vector3 normal( nx, ny, nz );
    if ( normal.isNaN() )
      continue;
    const Ogre::Vector3 pt2 = pt + normal.normalisedCopy() * normal_length_;

    manual_object_->position( pt );
    manual_object_->colour( color_ );
    manual_object_->position( pt2 );
    manual_object_->colour( color_ );
    bounding_box.merge( pt );
    bounding_box.merge( pt2 );
    index_count += 2;
  }
  manual_object_->end();
  manual_object_->setBoundingBox( bounding_box );
}

} // namespace hector_rviz_plugins
