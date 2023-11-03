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

namespace rviz {
    class BoolProperty;

    class FloatProperty;

    class EnumProperty;

    class TfFrameProperty;
} // namespace rviz

namespace pcl {
    template<class T>
    class PassThrough;

    template<class T>
    class PointCloud;
} // namespace pcl

namespace hector_rviz_plugins {

    class PointCloudFilterDisplay : public rviz::MessageFilterDisplay<sensor_msgs::PointCloud2> {
    Q_OBJECT
    public:
        PointCloudFilterDisplay();

        ~PointCloudFilterDisplay() override;

        void reset() override;

        void update(float wall_dt, float ros_dt) override;

    protected:
        void onInitialize() override;

        void processMessage(const sensor_msgs::PointCloud2ConstPtr& msg) override;

        std::string selectedFrame_;
        std::string selectedAxis_;
        bool filtering_;
        bool radialFiltering_;
        bool xFiltering_;
        bool yFiltering_;
        bool zFiltering_;

        rviz::BoolProperty *filterProperty_;
        rviz::BoolProperty *radialFilterProperty_;
        rviz::BoolProperty *xFilterProperty_;
        rviz::BoolProperty *yFilterProperty_;
        rviz::BoolProperty *zFilterProperty_;

        rviz::FloatProperty *maxRadialDistanceProperty_;
        rviz::FloatProperty *xMinValueProperty_;
        rviz::FloatProperty *xMaxValueProperty_;
        rviz::FloatProperty *yMinValueProperty_;
        rviz::FloatProperty *yMaxValueProperty_;
        rviz::FloatProperty *zMinValueProperty_;
        rviz::FloatProperty *zMaxValueProperty_;
        rviz::TfFrameProperty *frameProperty_;

        typedef std::deque<sensor_msgs::PointCloud2ConstPtr> D_CloudInfo;
        D_CloudInfo cloudQ_;

        rviz::PointCloudCommon *pointCloudCommon_;

        enum Axis {
            X, Y, Z
        };

    private Q_SLOTS:

        void updateParameters();

        void enableFiltering();
    };

} // namespace hector_rviz_plugins

#endif
