/*
 * Copyright (C) 2020  Jasper Suess
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

#ifndef HECTOR_RVIZ_PLUGINS_POINTCLOUD_FILTER_DISPLAY_H
#define HECTOR_RVIZ_PLUGINS_POINTCLOUD_FILTER_DISPLAY_H

#include <rviz/message_filter_display.h>

#include <deque>
#include <rviz/default_plugin/point_cloud_common.h>

namespace rviz
{
class BoolProperty;
class FloatProperty;
class EnumProperty;
class TfFrameProperty;
} // namespace rviz

namespace pcl
{
template<class T>
class PassThrough;
template<class T>
class PointCloud;
} // namespace pcl

namespace hector_rviz_plugins
{

class PointCloudFilterDisplay : public rviz::MessageFilterDisplay<sensor_msgs::PointCloud2>
{
  Q_OBJECT
public:
  PointCloudFilterDisplay();
  ~PointCloudFilterDisplay() override;

  void reset() override;

  void update( float wall_dt, float ros_dt ) override;

protected:
  void onInitialize() override;

  void processMessage( const sensor_msgs::PointCloud2ConstPtr &cloud ) override;

  template<class T>
  void filterCloud( pcl::PassThrough<T> filter, typename pcl::PointCloud<T>::Ptr cloud );

  std::string selected_frame;
  std::string selected_axis;
  bool filtering;

  rviz::BoolProperty *filter_property_;
  rviz::EnumProperty *axis_property_;
  rviz::FloatProperty *min_value_property_;
  rviz::FloatProperty *max_value_property_;
  rviz::TfFrameProperty *frame_property_;

  typedef std::deque<sensor_msgs::PointCloud2ConstPtr> D_CloudInfo;
  D_CloudInfo cloud_q;

  rviz::PointCloudCommon *point_cloud_common_;

  enum Axis { X, Y, Z };

private Q_SLOTS:
  void updateParameters();
  void enableFiltering();
};

} // namespace hector_rviz_plugins

#endif
