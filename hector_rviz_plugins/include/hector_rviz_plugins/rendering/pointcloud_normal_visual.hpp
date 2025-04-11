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

#ifndef HECTOR_RVIZ_PLUGINS_POINTCLOUD_NORMAL_VISUALIZATION_HPP
#define HECTOR_RVIZ_PLUGINS_POINTCLOUD_NORMAL_VISUALIZATION_HPP

#include <OgreColourValue.h>
#include <OgreSharedPtr.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace Ogre
{
class SceneNode;
} // namespace Ogre

namespace hector_rviz_plugins
{
class PointCloudNormalVisual
{
public:
  PointCloudNormalVisual( Ogre::SceneNode *parent );
  PointCloudNormalVisual();

  void updateData( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg );

  void setFlatColor( const Ogre::ColourValue &color );

  void setNormalLength( float length );

  void setVisible( bool value );

private:
  void updateVisual();

  Ogre::ColourValue color_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_msg_;
  std::unique_ptr<Ogre::ManualObject> manual_object_;
  Ogre::SceneNode *parent_;
  Ogre::SceneNode *scene_node_;
  float normal_length_ = 0.05f;
};
} // namespace hector_rviz_plugins

#endif // HECTOR_RVIZ_PLUGINS_POINTCLOUD_NORMAL_VISUALIZATION_HPP
