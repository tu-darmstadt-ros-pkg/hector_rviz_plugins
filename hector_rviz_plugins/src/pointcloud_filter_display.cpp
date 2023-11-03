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
 *  GNU General Public Licen+se for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "hector_rviz_plugins/pointcloud_filter_display.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/tf_frame_property.h>

#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <boost/algorithm/string.hpp>


#include <rviz/validate_floats.h>
#include <rviz/display.h>
#include <rviz/message_filter_display.h>
#include <rviz/default_plugin/point_cloud_common.h>
#include <rviz/default_plugin/point_cloud_transformers.h>


namespace hector_rviz_plugins {
    PointCloudFilterDisplay::PointCloudFilterDisplay()
            : pointCloudCommon_(new rviz::PointCloudCommon(this)), cloudQ_() {
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

        filtering_ = false;
        radialFiltering_ = false;
        xFiltering_ = false;
        yFiltering_ = false;
        zFiltering_ = false;

        filterProperty_ = new rviz::BoolProperty("Enable filtering", filtering_, "Whether to enable filtering or not",
                                                 this, SLOT(enableFiltering()), this);

        radialFilterProperty_ = new rviz::BoolProperty("Radial Filter", radialFiltering_,
                                                       "Whether to enable radial filtering or not", filterProperty_,
                                                       SLOT(updateParameters()), this);
        maxRadialDistanceProperty_ = new rviz::FloatProperty("Radius", 5.0,
                                                             "Maximum distance from the origin for points to be displayed in meter.",
                                                             radialFilterProperty_, SLOT(updateParameters()), this);

        xFilterProperty_ = new rviz::BoolProperty("X Filter", xFiltering_,
                                                  "Activates X-Coord Filter", filterProperty_,
                                                  SLOT(updateParameters()), this);
        yFilterProperty_ = new rviz::BoolProperty("Y Filter", yFiltering_,
                                                  "Activates Y-Coord Filter", filterProperty_,
                                                  SLOT(updateParameters()), this);
        zFilterProperty_ = new rviz::BoolProperty("Z Filter", zFiltering_,
                                                  "Activates Z-Coord Filter", filterProperty_,
                                                  SLOT(updateParameters()), this);

        xMinValueProperty_ = new rviz::FloatProperty("X Min Value", -2.0, "Minimum value for points to be displayed.",
                                                     xFilterProperty_, SLOT(updateParameters()), this);
        xMaxValueProperty_ = new rviz::FloatProperty("X Max Value", 2.0, "Maximum value for points to be displayed.",
                                                     xFilterProperty_, SLOT(updateParameters()), this);
        yMinValueProperty_ = new rviz::FloatProperty("Y Min Value", -2.0, "Minimum value for points to be displayed.",
                                                     yFilterProperty_, SLOT(updateParameters()), this);
        yMaxValueProperty_ = new rviz::FloatProperty("Y Max Value", 2.0, "Maximum value for points to be displayed.",
                                                     yFilterProperty_, SLOT(updateParameters()), this);
        zMinValueProperty_ = new rviz::FloatProperty("Z Min Value", -2.0, "Minimum value for points to be displayed.",
                                                     zFilterProperty_, SLOT(updateParameters()), this);
        zMaxValueProperty_ = new rviz::FloatProperty("Z Max Value", 2.0, "Maximum value for points to be displayed.",
                                                     zFilterProperty_, SLOT(updateParameters()), this);

        frameProperty_ = new rviz::TfFrameProperty("Frame", "world",
                                                   "The frame to which the points are displayed relatively.",
                                                   filterProperty_, nullptr, true, SLOT(updateParameters()), this);
    }

    PointCloudFilterDisplay::~PointCloudFilterDisplay() = default;

    void PointCloudFilterDisplay::onInitialize() {
        MFDClass::onInitialize();
        update_nh_.setCallbackQueue(context_->getThreadedQueue());

        selectedFrame_ = context_->getFixedFrame().toStdString();
        frameProperty_->setFrameManager(context_->getFrameManager());
        frameProperty_->setStdString(selectedFrame_);
        pointCloudCommon_->initialize(context_, scene_node_);
    }

    void PointCloudFilterDisplay::processMessage(const sensor_msgs::PointCloud2ConstPtr &msg) {
        //Check whether pointcloud was already saved, if yes parameter update occurred.
        //Then, the cloud does not need to be registered again but just passed on to point_cloud_common with the new parameters.
        auto it = cloudQ_.begin();
        auto end = cloudQ_.end();

        bool known_cloud = false;

        for (; it != end; ++it) {
            if (it->get()->header.stamp == msg->header.stamp) {
                known_cloud = true;
                break;
            }
        }
        auto now_sec = ros::Time::now().toSec();

        // Check for old clouds outside of decay time
        // cloudQ_ is there so that in case of an accumulated pointcloud (decay_time > 0), the old data
        // of points filtered out is still available if the parameters for the filter change
        // later on. The decay_time is the time in seconds after which the old data is deleted.
        // it can be set in rviz.
        while (!cloudQ_.empty() && now_sec - cloudQ_.front()->header.stamp.toSec() >
                                   pointCloudCommon_->decay_time_property_->getFloat()) {
            if (cloudQ_.front()->header.stamp.toSec() == msg->header.stamp.toSec()) {
                cloudQ_.pop_front();
                return;
            }
            cloudQ_.pop_front();
        }

        //Transform cloud into the selected frame so x, y, z values can be filtered easily
        sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
        if (!context_->getFrameManager()->getTF2BufferPtr()->canTransform(
                selectedFrame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.0))) {
            ROS_DEBUG("Selected frame %s is not available!", selectedFrame_.c_str());
            return;
        }
        pcl_ros::transformPointCloud(selectedFrame_, *msg, *cloud, *context_->getFrameManager()->getTF2BufferPtr());

        // add the transformed cloud to the queue
        if (!known_cloud)
            cloudQ_.emplace_back(cloud);

        // Filter any nan values out of the cloud. Any nan values that make it through to PointCloudBase
        // will get their points put off in lala land, but it means they still do get processed/rendered
        // which can be a big performance hit
        sensor_msgs::PointCloud2Ptr filtered(new sensor_msgs::PointCloud2);

        int32_t xi = rviz::findChannelIndex(cloud, "x");
        int32_t yi = rviz::findChannelIndex(cloud, "y");
        int32_t zi = rviz::findChannelIndex(cloud, "z");

        if (xi == -1 || yi == -1 || zi == -1) {
            return;
        }

        const uint32_t xoff = cloud->fields[xi].offset;
        const uint32_t yoff = cloud->fields[yi].offset;
        const uint32_t zoff = cloud->fields[zi].offset;
        const uint32_t point_step = cloud->point_step;
        const size_t point_count = cloud->width * cloud->height;

        if (point_count * point_step != cloud->data.size()) {
            std::stringstream ss;
            ss << "Data size (" << cloud->data.size() << " bytes) does not match width (" << cloud->width
               << ") times height (" << cloud->height << ") times point_step (" << point_step
               << ").  Dropping message.";
            setStatusStd(rviz::StatusProperty::Error, "Message", ss.str());
            return;
        }
        filtered->data.resize(cloud->data.size());
        uint32_t output_count;

        if (point_count == 0) {
            output_count = 0;
        } else {
            uint8_t *output_ptr = &filtered->data.front();
            const uint8_t *ptr = &cloud->data.front(), *ptr_end = &cloud->data.back(), *ptr_init;
            size_t points_to_copy = 0;

            // Get the parameters for the filtering
            radialFiltering_ = radialFilterProperty_->getBool();
            xFiltering_ = xFilterProperty_->getBool();
            yFiltering_ = yFilterProperty_->getBool();
            zFiltering_ = zFilterProperty_->getBool();
            float maxRadialDist2 = maxRadialDistanceProperty_->getFloat() * maxRadialDistanceProperty_->getFloat();

            /*
            tfListener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);
            geometry_msgs::Transform trans = tfBuffer_.lookupTransform("world", frameId, ros::Time(0), ros::Duration(1)).transform;
            Eigen::Vector3d pos(trans.translation.x, trans.translation.y, trans.translation.z);
             */

            //Filter points out depending on the selected axis and the min and max values set and the radial distance
            for (; ptr < ptr_end; ptr += point_step) {
                float x = *reinterpret_cast<const float *>(ptr + xoff);
                float y = *reinterpret_cast<const float *>(ptr + yoff);
                float z = *reinterpret_cast<const float *>(ptr + zoff);

                // Check for NaNs. If any of x,y,z is a NaN, skip the whole point.
                bool addPoint = true;
                if (rviz::validateFloats(x) && rviz::validateFloats(y) && rviz::validateFloats(z)) {
                    // Check for min and max values for the selected axis and radial distance
                    if(xFiltering_){
                        addPoint = x < xMaxValueProperty_->getFloat() && x > xMinValueProperty_->getFloat();
                    }
                    if(addPoint && yFiltering_){
                        addPoint = y < yMaxValueProperty_->getFloat() && y > yMinValueProperty_->getFloat();
                    }
                    if(addPoint && zFiltering_){
                        addPoint = z < zMaxValueProperty_->getFloat() && z > zMinValueProperty_->getFloat();
                    }
                    if (addPoint && radialFiltering_) {
                        addPoint = x * x + y * y + z * z < maxRadialDist2;
                    }
                } else {
                    addPoint = false;
                }

                if (addPoint) {
                    if (points_to_copy == 0) {
                        // Only memorize where to start copying from
                        ptr_init = ptr;
                        points_to_copy = 1;
                    } else {
                        ++points_to_copy;
                    }
                } else {
                    if (points_to_copy) {
                        // Copy all the points that need to be copied
                        memcpy(output_ptr, ptr_init, point_step * points_to_copy);
                        output_ptr += point_step * points_to_copy;
                        points_to_copy = 0;
                    }
                }
            }
            // Don't forget to flush what needs to be copied
            if (points_to_copy) {
                memcpy(output_ptr, ptr_init, point_step * points_to_copy);
                output_ptr += point_step * points_to_copy;
            }
            output_count = (output_ptr - &filtered->data.front()) / point_step;
        }

        filtered->header = cloud->header;
        filtered->fields = cloud->fields;
        filtered->data.resize(output_count * point_step);
        filtered->height = 1;
        filtered->width = output_count;
        filtered->is_bigendian = cloud->is_bigendian;
        filtered->point_step = point_step;
        filtered->row_step = output_count;
        pointCloudCommon_->addMessage(filtered);
    }

    void PointCloudFilterDisplay::update(float wall_dt, float ros_dt) {
        pointCloudCommon_->update(wall_dt, ros_dt);
    }

    void PointCloudFilterDisplay::reset() {
        MFDClass::reset();
        pointCloudCommon_->reset();
        cloudQ_.clear();
    }

    void PointCloudFilterDisplay::updateParameters() {
        pointCloudCommon_->reset();

        if (frameProperty_->getStdString() == "<Fixed Frame>")
            selectedFrame_ = context_->getFixedFrame().toStdString();
        else
            selectedFrame_ = frameProperty_->getFrameStd();

        // Process each saved cloud again with the changed parameters and pass to point_cloud_common
        auto it = cloudQ_.begin();
        auto end = cloudQ_.end();

        for (; it != end; ++it) {
            const sensor_msgs::PointCloud2ConstPtr &cloud_msg = *it;
            processMessage(cloud_msg);
        }
    }

    void PointCloudFilterDisplay::enableFiltering() {
        filtering_ = filterProperty_->getBool();

        if (filtering_)
            filterProperty_->expand();

        updateParameters();
    }

} // namespace hector_rviz_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hector_rviz_plugins::PointCloudFilterDisplay, rviz::Display)
