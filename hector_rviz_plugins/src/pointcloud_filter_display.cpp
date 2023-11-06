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
                                                   "The frame to which the points are filtered to relatively.",
                                                   filterProperty_, nullptr, true, SLOT(updateParameters()), this);

        useAxesFrameProperty_ = new rviz::BoolProperty("Use other Axes", false,
                                                       "Whether to use a different Frame for the axes that filtering is based on",
                                                       filterProperty_, SLOT(updateParameters()), this);
        axesFrameProperty_ = new rviz::TfFrameProperty("Axes Frame", "world",
                                                       "The frame used for the filter axes.",
                                                       useAxesFrameProperty_, nullptr, true, SLOT(updateParameters()), this);
    }

    PointCloudFilterDisplay::~PointCloudFilterDisplay() = default;

    void PointCloudFilterDisplay::onInitialize() {
        MFDClass::onInitialize();
        update_nh_.setCallbackQueue(context_->getThreadedQueue());

        selectedFrame_ = context_->getFixedFrame().toStdString();
        frameProperty_->setFrameManager(context_->getFrameManager());
        frameProperty_->setStdString(selectedFrame_);
        axesFrameProperty_->setFrameManager(context_->getFrameManager());
        axesFrameProperty_->setStdString(selectedFrame_);
        pointCloudCommon_->initialize(context_, scene_node_);
    }

    void PointCloudFilterDisplay::processMessage(const sensor_msgs::PointCloud2ConstPtr &msg) {
        //Check whether pointcloud was already saved, if yes parameter update occurred.
        //Then, the cloud does not need to be registered again but just filtered with the new parameters.
        bool knownCloud = false;
        auto it = cloudQ_.begin();
        while (it != cloudQ_.end()) {
            if (it->get()->header.stamp == msg->header.stamp) {
                knownCloud = true;
                break;
            }
            ++it;
        }

        // Check for old clouds outside of decay time
        auto nowSec = ros::Time::now().toSec();
        while (!cloudQ_.empty() && nowSec - cloudQ_.front()->header.stamp.toSec() >
                                   pointCloudCommon_->decay_time_property_->getFloat()) {
            if (cloudQ_.front()->header.stamp.toSec() == msg->header.stamp.toSec()) {
                cloudQ_.pop_front();
                return;
            }
            cloudQ_.pop_front();
        }

        if (!knownCloud)
            cloudQ_.emplace_back(msg);

        if (filtering_) {
            sensor_msgs::PointCloud2Ptr filtered = filterPointCloud(msg);
            if (filtered) {
                pointCloudCommon_->addMessage(filtered);
            }
        } else {
            pointCloudCommon_->addMessage(msg);
        }
    }

    sensor_msgs::PointCloud2Ptr PointCloudFilterDisplay::filterPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg) {
        sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
        std::string targetFrame;
        bool useAxesFrame = useAxesFrameProperty_->getBool();

        if (useAxesFrame) {
            // PointCloud gets transformed into the world frame and filtered relative to the position of the selected frame
            if (axesFrameProperty_->getStdString() == "<Fixed Frame>")
                targetFrame = context_->getFixedFrame().toStdString();
            else
                targetFrame = axesFrameProperty_->getFrameStd();

            if (!context_->getFrameManager()->getTF2BufferPtr()->canTransform(
                    targetFrame, selectedFrame_, ros::Time(0), ros::Duration(0.0))) {
                ROS_DEBUG("Selected frame %s is not available!", selectedFrame_.c_str());
                return nullptr;
            }
        } else {
            // PointCloud gets transformed into the selected frame and filtered
            targetFrame = selectedFrame_;
            if (!context_->getFrameManager()->getTF2BufferPtr()->canTransform(
                    selectedFrame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.0))) {
                ROS_DEBUG("Selected frame %s is not available!", selectedFrame_.c_str());
                return nullptr;
            }
        }
        pcl_ros::transformPointCloud(targetFrame, *msg, *cloud, *context_->getFrameManager()->getTF2BufferPtr());

        // Filter any nan values out of the cloud. Any nan values that make it through to PointCloudBase
        // will get their points put off in lala land, but it means they still do get processed/rendered
        // which can be a big performance hit
        sensor_msgs::PointCloud2Ptr filtered(new sensor_msgs::PointCloud2);

        int32_t xi = rviz::findChannelIndex(cloud, "x");
        int32_t yi = rviz::findChannelIndex(cloud, "y");
        int32_t zi = rviz::findChannelIndex(cloud, "z");
        if (xi == -1 || yi == -1 || zi == -1) {
            return nullptr;
        }
        const uint32_t xOff = cloud->fields[xi].offset;
        const uint32_t yOff = cloud->fields[yi].offset;
        const uint32_t zOff = cloud->fields[zi].offset;
        const uint32_t pointStep = cloud->point_step;
        const size_t pointCount = cloud->width * cloud->height;

        if (pointCount * pointStep != cloud->data.size()) {
            std::stringstream ss;
            ss << "Data size (" << cloud->data.size() << " bytes) does not match width (" << cloud->width
               << ") times height (" << cloud->height << ") times pointStep (" << pointStep
               << ").  Dropping message.";
            setStatusStd(rviz::StatusProperty::Error, "Message", ss.str());
            return nullptr;
        }
        filtered->data.resize(cloud->data.size());
        uint32_t outputCount;

        if (pointCount == 0) {
            outputCount = 0;
        } else {
            uint8_t *outputPtr = &filtered->data.front();
            const uint8_t *ptr = &cloud->data.front();
            const uint8_t *ptrEnd = &cloud->data.back();
            const uint8_t *ptrInit;
            size_t pointsToCopy = 0;

            // Get the parameters for the filtering
            radialFiltering_ = radialFilterProperty_->getBool();
            xFiltering_ = xFilterProperty_->getBool();
            yFiltering_ = yFilterProperty_->getBool();
            zFiltering_ = zFilterProperty_->getBool();
            float maxRadialDist2 = maxRadialDistanceProperty_->getFloat() * maxRadialDistanceProperty_->getFloat();

            Eigen::Vector3f relativePos(0, 0, 0);
            if(useAxesFrame){
                geometry_msgs::Transform trans = context_->getFrameManager()->getTF2BufferPtr()->lookupTransform(
                        targetFrame, selectedFrame_, ros::Time(0), ros::Duration(0.0)).transform;
                relativePos = {(float)trans.translation.x, (float)trans.translation.y, (float)trans.translation.z};
            }

            //Filter points out depending on the selected axis and the min and max values set and the radial distance
            for (; ptr < ptrEnd; ptr += pointStep) {
                float x = *reinterpret_cast<const float *>(ptr + xOff);
                float y = *reinterpret_cast<const float *>(ptr + yOff);
                float z = *reinterpret_cast<const float *>(ptr + zOff);

                // Check for NaNs. If any of x,y,z is a NaN, skip the whole point.
                bool addPoint = true;
                if (rviz::validateFloats(x) && rviz::validateFloats(y) && rviz::validateFloats(z) && filtering_) {
                    if(useAxesFrame){
                        x -= relativePos.x();
                        y -= relativePos.y();
                        z -= relativePos.z();
                    }
                    // Check for min and max values for the selected axis and radial distance
                    if (xFiltering_) {
                        addPoint = x < xMaxValueProperty_->getFloat() && x > xMinValueProperty_->getFloat();
                    }
                    if (addPoint && yFiltering_) {
                        addPoint = y < yMaxValueProperty_->getFloat() && y > yMinValueProperty_->getFloat();
                    }
                    if (addPoint && zFiltering_) {
                        addPoint = z < zMaxValueProperty_->getFloat() && z > zMinValueProperty_->getFloat();
                    }
                    if (addPoint && radialFiltering_) {
                        addPoint = x * x + y * y + z * z < maxRadialDist2;
                    }
                } else {
                    addPoint = false;
                }

                if (addPoint) {
                    if (pointsToCopy == 0) {
                        // Only memorize where to start copying from
                        ptrInit = ptr;
                        pointsToCopy = 1;
                    } else {
                        ++pointsToCopy;
                    }
                } else {
                    if (pointsToCopy) {
                        // Copy all the points that need to be copied
                        memcpy(outputPtr, ptrInit, pointStep * pointsToCopy);
                        outputPtr += pointStep * pointsToCopy;
                        pointsToCopy = 0;
                    }
                }
            }
            // Don't forget to flush what needs to be copied
            if (pointsToCopy) {
                memcpy(outputPtr, ptrInit, pointStep * pointsToCopy);
                outputPtr += pointStep * pointsToCopy;
            }
            outputCount = (outputPtr - &filtered->data.front()) / pointStep;
        }

        filtered->header = cloud->header;
        filtered->fields = cloud->fields;
        filtered->data.resize(outputCount * pointStep);
        filtered->height = 1;
        filtered->width = outputCount;
        filtered->is_bigendian = cloud->is_bigendian;
        filtered->point_step = pointStep;
        filtered->row_step = outputCount;
        return filtered;
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
