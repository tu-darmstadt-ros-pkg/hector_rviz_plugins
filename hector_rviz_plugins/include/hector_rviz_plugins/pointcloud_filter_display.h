#ifndef HECTOR_RVIZ_PLUGINS_POINTCLOUD_FILTER_DISPLAY_H
#define HECTOR_RVIZ_PLUGINS_POINTCLOUD_FILTER_DISPLAY_H

#include <sensor_msgs/PointCloud2.h>

#include <rviz/display.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/frame_manager.h>
#include <rviz/default_plugin/point_cloud_common.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <tf/transform_listener.h>
#include <deque>


namespace hector_rviz_plugins
{

class PointCloudFilterDisplay : public rviz::MessageFilterDisplay<sensor_msgs::PointCloud2>
{
  Q_OBJECT
public:
  PointCloudFilterDisplay();
  ~PointCloudFilterDisplay() override;

  void reset() override;

  void update(float wall_dt, float ros_dt) override;


protected:
  void onInitialize() override;

  void processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud) override;

  tf::TransformListener *tf_listener = new tf::TransformListener(ros::Duration(100));
  rviz::FrameManager *frame_manager_ = new rviz::FrameManager();

  std::string selected_frame;
  bool filtering;

  rviz::BoolProperty* filter_property_;
  rviz::EnumProperty* axis_property_;
  rviz::FloatProperty* min_value_property_;
  rviz::FloatProperty* max_value_property_;
  rviz::TfFrameProperty* frame_property_;

  typedef std::deque<sensor_msgs::PointCloud2ConstPtr> D_CloudInfo;
  D_CloudInfo cloud_q;

  rviz::PointCloudCommon* point_cloud_common_;

  enum Axis{
      X,
      Y,
      Z
    };


private Q_SLOTS:
    void updateParameters();
    void enableFiltering();
};

} // namespace hector_rviz_plugins

#endif
