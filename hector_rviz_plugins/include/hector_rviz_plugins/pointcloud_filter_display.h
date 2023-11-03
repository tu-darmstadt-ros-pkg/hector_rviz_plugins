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

        /**
         * @brief reset
         * This function is called when the reset button in rviz is pressed. It clears the queue and resets the
         * pointCloudCommon.
         */
        void reset() override;

        /**
         * @brief update
         * This function is called periodically by rviz. It is used to update the display.
         * @param wall_dt
         * @param ros_dt
         */
        void update(float wall_dt, float ros_dt) override;

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
        sensor_msgs::PointCloud2Ptr filterPointCloud(const sensor_msgs::PointCloud2Ptr &cloud);

        /**
         * @brief processMessage
         * This function is called when a new PointCloud2 message is received. It is the callback function
         * It adds the message to the queue, calls the filterPointCloud function and publishes the filtered message.
         * @param msg The PointCloud2 message
         */
        void processMessage(const sensor_msgs::PointCloud2ConstPtr& msg) override;

        std::string selectedFrame_;
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
        /*
         * cloudQ_ is there so that in case of an accumulated pointCloud (decay_time > 0), the old data
         * of points filtered out is still available if the parameters for the filter are changed.
         * The decay_time is the time in seconds after which the old data is deleted. It can be set in rviz.
         */
        D_CloudInfo cloudQ_;
        rviz::PointCloudCommon *pointCloudCommon_;

    private Q_SLOTS:

        void updateParameters();

        void enableFiltering();
    };

} // namespace hector_rviz_plugins

#endif
