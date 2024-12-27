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

#ifndef HECTOR_RVIZ_PLUGINS_POINTCLOUD_FILTER_DISPLAY_HPP
#define HECTOR_RVIZ_PLUGINS_POINTCLOUD_FILTER_DISPLAY_HPP

#include <deque>
#include <rviz_common/message_filter_display.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rviz_common::properties
{
class FloatProperty;
class TfFrameProperty;
} // namespace rviz_common::properties

namespace rviz_default_plugins
{
class PointCloudCommon;
}

namespace hector_rviz_plugins
{

class PointCloudFilterDisplay
    : public rviz_common::MessageFilterDisplay<sensor_msgs::msg::PointCloud2>
{
  Q_OBJECT
public:
  PointCloudFilterDisplay();

  ~PointCloudFilterDisplay() override;

  /**
   * @brief reset
   * This function is called when the reset button in rviz is pressed. It clears the queue and
   * resets the pointCloudCommon.
   */
  void reset() override;

  /**
   * @brief update
   * This function is called periodically by rviz. It is used to update the display.
   * @param wall_dt
   * @param ros_dt
   */
  void update( float wall_dt, float ros_dt ) override;

  bool isFilterActive() const;

protected:
  /**
   * @brief onInitialize
   * This function is called when the display is initialized. It sets up the properties and the FrameManager.
   */
  void onInitialize() override;

  /**
   * @brief filterPointCloud
   * This function filters the pointCloud depending on the parameters set in rviz.
   * @param cloud The PointCloud2 message
   * @return The filtered PointCloud2 message
   */
  sensor_msgs::msg::PointCloud2::SharedPtr
  filterPointCloud( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg );

  bool getTransform( const std::string &source_frame, const tf2::TimePoint &time_point,
                     Ogre::Vector3 &translation, Ogre::Quaternion &orientation );

  /**
   * @brief processMessage
   * This function is called when a new PointCloud2 message is received. It is the callback function
   * It adds the message to the queue, calls the filterPointCloud function and publishes the filtered message.
   * @param msg The PointCloud2 message
   */
  void processMessage( sensor_msgs::msg::PointCloud2::ConstSharedPtr msg ) override;

  rviz_common::properties::BoolProperty *filter_group_property_;
  rviz_common::properties::BoolProperty *radial_filter_property_;
  rviz_common::properties::BoolProperty *x_filter_property_;
  rviz_common::properties::BoolProperty *y_filter_property_;
  rviz_common::properties::BoolProperty *z_filter_property_;
  rviz_common::properties::BoolProperty *use_axes_frame_property_;
  rviz_common::properties::BoolProperty *filter_by_channel_value_property_;

  rviz_common::properties::FloatProperty *max_radial_distance_property_;
  rviz_common::properties::FloatProperty *x_min_value_property_;
  rviz_common::properties::FloatProperty *x_max_value_property_;
  rviz_common::properties::FloatProperty *y_min_value_property_;
  rviz_common::properties::FloatProperty *y_max_value_property_;
  rviz_common::properties::FloatProperty *z_min_value_property_;
  rviz_common::properties::FloatProperty *z_max_value_property_;
  rviz_common::properties::FloatProperty *channel_max_value_property_;
  rviz_common::properties::FloatProperty *channel_min_value_property_;
  rviz_common::properties::EditableEnumProperty *channel_property_;

  rviz_common::properties::TfFrameProperty *frame_property_;
  rviz_common::properties::TfFrameProperty *axes_frame_property_;

  /*
   * cloud_queue_ is there so that in case of an accumulated pointCloud (decay_time > 0), the old data
   * of points filtered out is still available if the parameters for the filter are changed.
   * The decay_time is the time in seconds after which the old data is deleted. It can be set in rviz.
   */
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloud_queue_;
  std::unique_ptr<rviz_default_plugins::PointCloudCommon> point_cloud_common_;

private Q_SLOTS:

  void updateParameters();

  void enableFiltering();
};

} // namespace hector_rviz_plugins

#endif
