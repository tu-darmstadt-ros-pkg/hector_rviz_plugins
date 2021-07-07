#include "hector_rviz_plugins/pointcloud_filter_display.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/frame_manager.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

#include <boost/algorithm/string.hpp>


namespace hector_rviz_plugins
{
PointCloudFilterDisplay::PointCloudFilterDisplay() : point_cloud_common_(new rviz::PointCloudCommon(this)), cloud_q()
{
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    filtering = false;
    selected_axis = "z";

    filter_property_ = new rviz::BoolProperty("Enable filtering", filtering,
                             "Whether to enable filtering or not",
                             this, SLOT(enableFiltering()), this);

    axis_property_ = new rviz::EnumProperty( "Filtered Axis", "Z",
                                               "Axis around which the points are filtered",
                                               filter_property_, SLOT( updateParameters()), this);

    axis_property_->addOption("X", X);
    axis_property_->addOption("Y", Y);
    axis_property_->addOption("Z", Z);


    min_value_property_ = new rviz::FloatProperty( "Min Value", -2.0,
                                                  "Minimum value for points to be displayed.",
                                                   filter_property_, SLOT( updateParameters()), this);

    max_value_property_ = new rviz::FloatProperty( "Max Value", 2.0,
                                                   "Maximum value for points to be displayed.",
                                                   filter_property_, SLOT( updateParameters()), this);

    frame_property_ = new rviz::TfFrameProperty("Frame", nullptr,
                                           "The frame to which the points are displayed relatively.",
                                                filter_property_, nullptr, true, SLOT( updateParameters()), this);


}

    PointCloudFilterDisplay::~PointCloudFilterDisplay() = default;

void PointCloudFilterDisplay::onInitialize()
{
    MFDClass::onInitialize();
    update_nh_.setCallbackQueue(context_->getThreadedQueue());

    selected_frame = context_->getFixedFrame().toStdString();
    frame_property_->setFrameManager(context_->getFrameManager());
    frame_property_->setStdString(selected_frame);

    point_cloud_common_->initialize(context_, scene_node_);

}

void PointCloudFilterDisplay::processMessage(const sensor_msgs::PointCloud2ConstPtr& msg)
{

    //Check whether pointcloud was already saved, if yes parameter update occurred.
    //Then, the cloud does not need to be registered again but just passed on to point_cloud_common with the new parameters.
    auto it = cloud_q.begin();
    auto end = cloud_q.end();

    bool known_cloud = false;

    for (; it != end; ++it) {

        if(it->get()->header.stamp == msg->header.stamp){
            known_cloud = true;
            break;
        }
    }
    auto now_sec = ros::Time::now().toSec();

    //Check for old clouds outside of decay time
    while(!cloud_q.empty() && now_sec - cloud_q.front()->header.stamp.toSec() > point_cloud_common_->decay_time_property_->getFloat())
    {
        if(cloud_q.front()->header.stamp.toSec() == msg->header.stamp.toSec()){
            cloud_q.pop_front();
            return;
        }
        cloud_q.pop_front();
    }

    //Transform cloud into the selected frame so x, y, z values can be filtered easily
    sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);

    if(!context_->getFrameManager()->getTF2BufferPtr()->canTransform(selected_frame, msg->header.frame_id, msg->header.stamp, ros::Duration(0.0))){
        ROS_DEBUG("Selected frame %s is not available!", selected_frame.c_str());
        return;
    }

    pcl_ros::transformPointCloud(selected_frame, *msg, *cloud, *context_->getFrameManager()->getTF2BufferPtr());

    if(!known_cloud)
        cloud_q.push_back(cloud);

    if(filtering) {

        //Decide between colored clouds and clouds with intensity information
        if(sensor_msgs::getPointCloud2FieldIndex(*msg, "rgb") != -1){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PassThrough<pcl::PointXYZRGB> pass;

            pcl::fromROSMsg(*cloud, *pcl_cloud);
            filterCloud(pass, pcl_cloud);
        }
        else if(sensor_msgs::getPointCloud2FieldIndex(*msg, "intensity") != -1){
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PassThrough<pcl::PointXYZI> pass;

            pcl::fromROSMsg(*cloud, *pcl_cloud);
            filterCloud(pass, pcl_cloud);
        }
        else{
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PassThrough<pcl::PointXYZ> pass;

            pcl::fromROSMsg(*cloud, *pcl_cloud);
            filterCloud(pass, pcl_cloud);
        }
    }
    else{
        point_cloud_common_->addMessage(msg);
    }
}

template <class T>
void PointCloudFilterDisplay::filterCloud(pcl::PassThrough<T> filter, typename pcl::PointCloud<T>::Ptr cloud){

    sensor_msgs::PointCloud2Ptr cloud_filtered(new sensor_msgs::PointCloud2);

    filter.setInputCloud(cloud);
    filter.setFilterFieldName(selected_axis);
    filter.setFilterLimits(min_value_property_->getFloat(), max_value_property_->getFloat());

    filter.filter(*cloud);

    pcl::toROSMsg(*cloud, *cloud_filtered);
    point_cloud_common_->addMessage(cloud_filtered);
}

void PointCloudFilterDisplay::update(float wall_dt, float ros_dt)
{
    point_cloud_common_->update(wall_dt, ros_dt);
}

void PointCloudFilterDisplay::reset()
{
    MFDClass::reset();
    point_cloud_common_->reset();
    cloud_q.clear();

}

void PointCloudFilterDisplay::updateParameters(){
    point_cloud_common_->reset();

    if(frame_property_->getStdString() == "<Fixed Frame>")
        selected_frame = context_->getFixedFrame().toStdString();
    else
        selected_frame = frame_property_->getFrameStd();

    selected_axis = axis_property_->getStdString();
    boost::algorithm::to_lower(selected_axis);

    //Process each saved cloud again with the changed parameters and pass to point_cloud_common
    auto it = cloud_q.begin();
    auto end = cloud_q.end();

    for (; it != end; ++it) {

        const sensor_msgs::PointCloud2ConstPtr & cloud_msg = *it;
        processMessage(cloud_msg);
    }
}

void PointCloudFilterDisplay::enableFiltering(){
        filtering = filter_property_->getBool();

        if(filtering)
            filter_property_->expand();

        updateParameters();
    }

} // namespace hector_rviz_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hector_rviz_plugins::PointCloudFilterDisplay, rviz::Display)
