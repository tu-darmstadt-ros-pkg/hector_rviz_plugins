#include <ros/ros.h>
#include "hector_rviz_plugins/pointcloud_filter_display.h"

namespace hector_rviz_plugins
{
PointCloudFilterDisplay::PointCloudFilterDisplay() : point_cloud_common_(new rviz::PointCloudCommon(this)), cloud_q()
{
    point_cloud_common_->decay_time_property_->setMax(100);

    axis_property_ = new rviz::EnumProperty( "Axis", "z",
                                               "Axis around the points are filtered",
                                               this, SLOT( updateParameters()));

    axis_property_->addOption("x", X);
    axis_property_->addOption("y", Y);
    axis_property_->addOption("z", Z);


    min_value_property_ = new rviz::FloatProperty( "Min Value", -2.0,
                                                  "Minimum value for points to be displayed.",
                                                  this, SLOT( updateParameters()));

    max_value_property_ = new rviz::FloatProperty( "Max Value", 2.0,
                                                   "Maximum value for points to be displayed.",
                                                   this, SLOT( updateParameters()));

    frame_property_ = new rviz::StringProperty( "Frame", "world",
                                               "The frame to which the points are displayed relatively.",
                                               this, SLOT( updateParameters()));

    update_nh_.setCallbackQueue(point_cloud_common_->getCallbackQueue());

}

    PointCloudFilterDisplay::~PointCloudFilterDisplay() = default;

void PointCloudFilterDisplay::onInitialize()
{
    MFDClass::onInitialize();
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

    if(tf_listener->waitForTransform(frame_property_->getStdString(), msg->header.frame_id, msg->header.stamp, ros::Duration(0.01))){
        pcl_ros::transformPointCloud(frame_property_->getStdString(), *msg, *cloud, *tf_listener);
    }
    else{
        ROS_DEBUG("Given frame does not exist!");
        pcl_ros::transformPointCloud("world", *msg, *cloud, *tf_listener);
    }

    if(!known_cloud)
        cloud_q.push_back(cloud);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2Ptr cloud_filtered(new sensor_msgs::PointCloud2);

    pcl::fromROSMsg(*cloud, *pcl_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName (axis_property_->getStdString());
    pass.setFilterLimits (min_value_property_->getFloat(), max_value_property_->getFloat());

    pass.filter(*pcl_cloud_filtered );

    pcl::toROSMsg(*pcl_cloud_filtered, *cloud_filtered);
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

}

void PointCloudFilterDisplay::updateParameters(){
    point_cloud_common_->reset();

    //Process each saved cloud again with the changed parameters and pass to point_cloud_common
    auto it = cloud_q.begin();
    auto end = cloud_q.end();

    for (; it != end; ++it) {

        const sensor_msgs::PointCloud2ConstPtr & cloud_msg = *it;
        processMessage(cloud_msg);
    }
}

} // namespace hector_rviz_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hector_rviz_plugins::PointCloudFilterDisplay, rviz::Display)
