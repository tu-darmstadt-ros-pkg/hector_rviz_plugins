#ifndef HECTOR_RVIZ_PLUGINS_POINTCLOUD_FILTER_DISPLAY_H
#define HECTOR_RVIZ_PLUGINS_POINTCLOUD_FILTER_DISPLAY_H

#include <rviz/message_filter_display.h>

#include <rviz/default_plugin/point_cloud_common.h>
#include <deque>

namespace rviz{
    class BoolProperty;
    class FloatProperty;
    class EnumProperty;
    class TfFrameProperty;
}

namespace pcl{
    template <class T> class PassThrough;
    template <class T> class PointCloud;
}

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

  template <class T>
  void filterCloud(pcl::PassThrough<T> filter, typename pcl::PointCloud<T>::Ptr cloud);

  std::string selected_frame;
  std::string selected_axis;
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
