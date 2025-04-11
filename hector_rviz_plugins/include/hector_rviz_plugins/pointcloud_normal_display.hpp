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

#ifndef HECTOR_RVIZ_PLUGINS_POINTCLOUD_NORMAL_DISPLAY_HPP
#define HECTOR_RVIZ_PLUGINS_POINTCLOUD_NORMAL_DISPLAY_HPP

#include <rviz_common/message_filter_display.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rviz_common::properties
{
class ColorProperty;
class FloatProperty;
}

namespace hector_rviz_plugins
{
class PointCloudNormalVisual;

class PointCloudNormalDisplay
    : public rviz_common::MessageFilterDisplay<sensor_msgs::msg::PointCloud2>
{
  Q_OBJECT
public:
  PointCloudNormalDisplay();

  ~PointCloudNormalDisplay() override;

  /**
   * @brief reset
   * This function is called when the reset button in rviz is pressed. It clears the queue and
   * resets the pointCloudCommon.
   */
  void reset() override;

  /**
   * This function is called periodically by rviz. It is used to update the display.
   */
  void update( float wall_dt, float ros_dt ) override;

  /**
   * Process the passed message and display the contained normals.
   * This function is called by the message filter when a new message arrives.
   */
  void processMessage( sensor_msgs::msg::PointCloud2::ConstSharedPtr msg ) override;

private slots:
  void updateColor();

  void updateNormalLength();

private:
  void onInitialize() override;

  std::unique_ptr<PointCloudNormalVisual> visual_;
  rviz_common::properties::ColorProperty *color_property_;
  rviz_common::properties::FloatProperty *normal_length_property_;
};
} // namespace hector_rviz_plugins

#endif // HECTOR_RVIZ_PLUGINS_POINTCLOUD_NORMAL_DISPLAY_HPP
