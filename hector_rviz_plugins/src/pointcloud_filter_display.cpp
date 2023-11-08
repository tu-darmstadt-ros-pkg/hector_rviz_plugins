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
            : point_cloud_common_(new rviz::PointCloudCommon(this)), cloud_q_() {
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

        filtering_ = false;
        radial_filtering_ = false;
        x_filtering_ = false;
        y_filtering_ = false;
        z_filtering_ = false;
        channel_filtering_ = false;

        filter_property_ = new rviz::BoolProperty("Enable filtering", filtering_, "Whether to enable filtering or not",
                                                  this, SLOT(enableFiltering()), this);

        radial_filter_property_ = new rviz::BoolProperty("Radial Filter", radial_filtering_,
                                                         "Whether to enable radial filtering or not", filter_property_,
                                                         SLOT(updateParameters()), this);
        max_radial_distance_property_ = new rviz::FloatProperty("Radius", 5.0,
                                                                "Maximum distance from the origin for points to be displayed in meter.",
                                                                radial_filter_property_, SLOT(updateParameters()),
                                                                this);

        x_filter_property_ = new rviz::BoolProperty("X Filter", x_filtering_,
                                                    "Activates X-Coord Filter", filter_property_,
                                                    SLOT(updateParameters()), this);
        y_filter_property_ = new rviz::BoolProperty("Y Filter", y_filtering_,
                                                    "Activates Y-Coord Filter", filter_property_,
                                                    SLOT(updateParameters()), this);
        z_filter_property_ = new rviz::BoolProperty("Z Filter", z_filtering_,
                                                    "Activates Z-Coord Filter", filter_property_,
                                                    SLOT(updateParameters()), this);
        filter_by_channel_value_property_ = new rviz::BoolProperty("Filter by Channel Value", channel_filtering_,
                                                                   "Whether to filter by channel value or not",
                                                                   filter_property_, SLOT(updateParameters()), this);
        channel_property_ = new rviz::StringProperty("Channel", nullptr,
                                                     "The channel to filter by", filter_by_channel_value_property_,
                                                     SLOT(updateParameters()), this);


        x_min_value_property_ = new rviz::FloatProperty("X Min Value", -2.0,
                                                        "Minimum value for points to be displayed.",
                                                        x_filter_property_, SLOT(updateParameters()), this);
        x_max_value_property_ = new rviz::FloatProperty("X Max Value", 2.0, "Maximum value for points to be displayed.",
                                                        x_filter_property_, SLOT(updateParameters()), this);
        y_min_value_property_ = new rviz::FloatProperty("Y Min Value", -2.0,
                                                        "Minimum value for points to be displayed.",
                                                        y_filter_property_, SLOT(updateParameters()), this);
        y_max_value_property_ = new rviz::FloatProperty("Y Max Value", 2.0, "Maximum value for points to be displayed.",
                                                        y_filter_property_, SLOT(updateParameters()), this);
        z_min_value_property_ = new rviz::FloatProperty("Z Min Value", -2.0,
                                                        "Minimum value for points to be displayed.",
                                                        z_filter_property_, SLOT(updateParameters()), this);
        z_max_value_property_ = new rviz::FloatProperty("Z Max Value", 2.0, "Maximum value for points to be displayed.",
                                                        z_filter_property_, SLOT(updateParameters()), this);
        channel_max_value_property_ = new rviz::FloatProperty("Channel Max Value", 255.0,
                                                              "Maximum value for points to be displayed.",
                                                              filter_by_channel_value_property_,
                                                              SLOT(updateParameters()), this);
        channel_min_value_property_ = new rviz::FloatProperty("Channel Min Value", -255.0,
                                                              "Minimum value for points to be displayed.",
                                                              filter_by_channel_value_property_,
                                                              SLOT(updateParameters()), this);

        frame_property_ = new rviz::TfFrameProperty("Frame", "world",
                                                    "The frame to which the points are filtered to relatively.",
                                                    filter_property_, nullptr, true, SLOT(updateParameters()), this);

        use_axes_frame_property_ = new rviz::BoolProperty("Use other Axes", false,
                                                          "Whether to use a different Frame for the axes that filtering is based on",
                                                          filter_property_, SLOT(updateParameters()), this);
        axes_frame_property_ = new rviz::TfFrameProperty("Axes Frame", "world",
                                                         "The frame used for the filter axes.",
                                                         use_axes_frame_property_, nullptr, true,
                                                         SLOT(updateParameters()), this);
    }

    PointCloudFilterDisplay::~PointCloudFilterDisplay() = default;

    void PointCloudFilterDisplay::onInitialize() {
        MFDClass::onInitialize();
        update_nh_.setCallbackQueue(context_->getThreadedQueue());

        selected_frame_ = context_->getFixedFrame().toStdString();
        frame_property_->setFrameManager(context_->getFrameManager());
        frame_property_->setStdString(selected_frame_);
        axes_frame_property_->setFrameManager(context_->getFrameManager());
        axes_frame_property_->setStdString(selected_frame_);
        point_cloud_common_->initialize(context_, scene_node_);
    }

    void PointCloudFilterDisplay::processMessage(const sensor_msgs::PointCloud2ConstPtr &msg) {
        //Check whether pointcloud was already saved, if yes parameter update occurred.
        //Then, the cloud does not need to be registered again but just filtered with the new parameters.
        bool known_cloud = false;
        auto it = cloud_q_.begin();
        while (it != cloud_q_.end()) {
            if (it->get()->header.stamp == msg->header.stamp) {
                known_cloud = true;
                break;
            }
            ++it;
        }

        // Check for old clouds outside of decay time
        auto now_sec = ros::Time::now().toSec();
        while (!cloud_q_.empty() && now_sec - cloud_q_.front()->header.stamp.toSec() >
                                    point_cloud_common_->decay_time_property_->getFloat()) {
            if (cloud_q_.front()->header.stamp.toSec() == msg->header.stamp.toSec()) {
                cloud_q_.pop_front();
                return;
            }
            cloud_q_.pop_front();
        }

        if (!known_cloud)
            cloud_q_.emplace_back(msg);

        if (filtering_) {
            sensor_msgs::PointCloud2Ptr filtered = filterPointCloud(msg);
            if (filtered) {
                point_cloud_common_->addMessage(filtered);
            }
        } else {
            point_cloud_common_->addMessage(msg);
        }
    }

    sensor_msgs::PointCloud2Ptr PointCloudFilterDisplay::filterPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg) {
        sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
        std::string target_frame;
        bool use_axes_frame = use_axes_frame_property_->getBool();

        if (use_axes_frame) {
            // PointCloud gets transformed into the world frame and filtered relative to the position of the selected frame
            if (axes_frame_property_->getStdString() == "<Fixed Frame>")
                target_frame = context_->getFixedFrame().toStdString();
            else
                target_frame = axes_frame_property_->getFrameStd();

            if (!context_->getFrameManager()->getTF2BufferPtr()->canTransform(
                    target_frame, selected_frame_, ros::Time(0), ros::Duration(0.0))) {
                ROS_DEBUG("Selected frame %s is not available!", selected_frame_.c_str());
                return nullptr;
            }
        } else {
            // PointCloud gets transformed into the selected frame and filtered
            target_frame = selected_frame_;
            if (!context_->getFrameManager()->getTF2BufferPtr()->canTransform(
                    selected_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.0))) {
                ROS_DEBUG("Selected frame %s is not available!", selected_frame_.c_str());
                return nullptr;
            }
        }
        pcl_ros::transformPointCloud(target_frame, *msg, *cloud, *context_->getFrameManager()->getTF2BufferPtr());

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

        // get active channel name
        bool filter_by_channel_value = filter_by_channel_value_property_->getBool();
        int32_t channel_i = rviz::findChannelIndex(cloud, channel_property_->getStdString());
        filter_by_channel_value = filter_by_channel_value && (channel_i != -1);
        uint32_t channel_off;
        if (filter_by_channel_value) {
            channel_off = cloud->fields[channel_i].offset;
        }
        
        const uint32_t x_off = cloud->fields[xi].offset;
        const uint32_t y_off = cloud->fields[yi].offset;
        const uint32_t z_off = cloud->fields[zi].offset;
        const uint32_t point_step = cloud->point_step;
        const size_t point_count = cloud->width * cloud->height;

        if (point_count * point_step != cloud->data.size()) {
            std::stringstream ss;
            ss << "Data size (" << cloud->data.size() << " bytes) does not match width (" << cloud->width
               << ") times height (" << cloud->height << ") times point_step (" << point_step
               << ").  Dropping message.";
            setStatusStd(rviz::StatusProperty::Error, "Message", ss.str());
            return nullptr;
        }
        filtered->data.resize(cloud->data.size());
        uint32_t output_count;

        if (point_count == 0) {
            output_count = 0;
        } else {
            uint8_t *output_ptr = &filtered->data.front();
            const uint8_t *ptr = &cloud->data.front();
            const uint8_t *ptr_end = &cloud->data.back();
            const uint8_t *ptr_init;
            size_t points_to_copy = 0;

            // Get the parameters for the filtering
            radial_filtering_ = radial_filter_property_->getBool();
            x_filtering_ = x_filter_property_->getBool();
            y_filtering_ = y_filter_property_->getBool();
            z_filtering_ = z_filter_property_->getBool();
            float max_radial_dist_2 =
                    max_radial_distance_property_->getFloat() * max_radial_distance_property_->getFloat();

            Eigen::Vector3f relativePos(0, 0, 0);
            if (use_axes_frame) {
                geometry_msgs::Transform trans = context_->getFrameManager()->getTF2BufferPtr()->lookupTransform(
                        target_frame, selected_frame_, ros::Time(0), ros::Duration(0.0)).transform;
                relativePos = {(float) trans.translation.x, (float) trans.translation.y, (float) trans.translation.z};
            }

            //Filter points out depending on the selected axis and the min and max values set and the radial distance
            for (; ptr < ptr_end; ptr += point_step) {
                float x = *reinterpret_cast<const float *>(ptr + x_off);
                float y = *reinterpret_cast<const float *>(ptr + y_off);
                float z = *reinterpret_cast<const float *>(ptr + z_off);
                float channel_value;
                if (filter_by_channel_value) {
                    channel_value = *reinterpret_cast<const float *>(ptr + channel_off);
                }

                // Check for NaNs. If any of x,y,z is a NaN, skip the whole point.
                bool add_point = true;
                if (rviz::validateFloats(x) && rviz::validateFloats(y) && rviz::validateFloats(z) && filtering_) {
                    if (use_axes_frame) {
                        x -= relativePos.x();
                        y -= relativePos.y();
                        z -= relativePos.z();
                    }
                    // Check for min and max values for the selected axis and radial distance
                    if (x_filtering_) {
                        add_point = x < x_max_value_property_->getFloat() && x > x_min_value_property_->getFloat();
                    }
                    if (add_point && y_filtering_) {
                        add_point = y < y_max_value_property_->getFloat() && y > y_min_value_property_->getFloat();
                    }
                    if (add_point && z_filtering_) {
                        add_point = z < z_max_value_property_->getFloat() && z > z_min_value_property_->getFloat();
                    }
                    if (add_point && filter_by_channel_value) {
                        add_point = channel_value < channel_max_value_property_->getFloat() &&
                                    channel_value > channel_min_value_property_->getFloat();
                    }
                    if (add_point && radial_filtering_) {
                        add_point = x * x + y * y + z * z < max_radial_dist_2;
                    }
                } else {
                    add_point = false;
                }

                if (add_point) {
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
        return filtered;
    }

    void PointCloudFilterDisplay::update(float wall_dt, float ros_dt) {
        point_cloud_common_->update(wall_dt, ros_dt);
    }

    void PointCloudFilterDisplay::reset() {
        MFDClass::reset();
        point_cloud_common_->reset();
        cloud_q_.clear();
    }

    void PointCloudFilterDisplay::updateParameters() {
        point_cloud_common_->reset();

        if (frame_property_->getStdString() == "<Fixed Frame>")
            selected_frame_ = context_->getFixedFrame().toStdString();
        else
            selected_frame_ = frame_property_->getFrameStd();

        // Process each saved cloud again with the changed parameters and pass to point_cloud_common
        auto it = cloud_q_.begin();
        auto end = cloud_q_.end();

        for (; it != end; ++it) {
            const sensor_msgs::PointCloud2ConstPtr &cloud_msg = *it;
            processMessage(cloud_msg);
        }
    }

    void PointCloudFilterDisplay::enableFiltering() {
        filtering_ = filter_property_->getBool();

        if (filtering_)
            filter_property_->expand();

        updateParameters();
    }

} // namespace hector_rviz_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hector_rviz_plugins::PointCloudFilterDisplay, rviz::Display)
