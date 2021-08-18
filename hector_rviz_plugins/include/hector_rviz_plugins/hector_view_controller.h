/*
 * Copyright (C) 2021  Stefan Fabian
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

#ifndef HECTOR_RVIZ_PLUGINS_HECTOR_VIEW_CONTROLLER_H
#define HECTOR_RVIZ_PLUGINS_HECTOR_VIEW_CONTROLLER_H

#include <rviz/view_controller.h>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <hector_rviz_plugins_msgs/SetViewMode.h>

namespace rviz
{
class BoolProperty;

class FloatProperty;

class Shape;

class TfFrameProperty;

class VectorProperty;
}

namespace hector_rviz_plugins
{

class CameraAnimator;

namespace view_modes
{
enum ViewMode
{
  Mode3D = 0,
  Mode2D = 1
};
}
using ViewMode = view_modes::ViewMode;

class HectorViewController
  : public rviz::ViewController
{
Q_OBJECT

public:
  HectorViewController();

  ~HectorViewController() override;

  void lookAt( const Ogre::Vector3 &point ) override;

  void mimic( rviz::ViewController *source_view ) override;

  void onInitialize() override;

  void reset() override;

  void handleMouseEvent( rviz::ViewportMouseEvent &evt ) override;

  void handleKeyEvent( QKeyEvent *event, rviz::RenderPanel *panel ) override;

  bool eventFilter( QObject *obj, QEvent *event ) override;

  void moveOnXYPlaneBy( float dx, float dy );

  void moveEyeWithFocusTo( const Ogre::Vector3 &eye, bool stop_tracking = true, bool animate = true,
                           bool switch_to_3d_mode = false );

  void moveEyeWithNewFocus( const Ogre::Vector3 &eye, const Ogre::Vector3 &focus, bool stop_tracking = true,
                            bool animate = true, bool switch_to_3d_mode = true );

  void cancelAnimation();

  void zoom( float ddistance );

  ViewMode mode() const;

  void setMode( ViewMode value, bool animate_transition = true );

  void trackFrame( const std::string &name );

  void stopTracking();

public Q_SLOTS:

  void onAnimationDurationChanged();

  void onEyePropertyChanged();

  void onFocusPointPropertyChanged();

  void onDistancePropertyChanged();

  void onTrackedFrameChanged();

  void onPGainChanged();

  void onMode2DChanged();

  void onKeyboardNavigationChanged();

  void onEnableTopicsChanged();

  void onEnableServicesChanged();

Q_SIGNALS:

  void modeChanged();

  void trackingChanged( bool tracking, const std::string &frame );

protected:
  void publishViewMode();

  void publishTrackedFrame();

  virtual void connectPositionProperties();

  virtual void disconnectPositionProperties();

  virtual void handleMouseEvent3D( rviz::ViewportMouseEvent &evt );

  virtual void handleMouseEvent2D( rviz::ViewportMouseEvent &evt );

  void update( float dt, float ros_dt ) override;

  virtual bool updateCameraProperties( float dt );

  virtual void updateCamera();

  virtual void updateDistance();

  virtual void updateOrientation( float delta_yaw, float delta_pitch, float delta_roll );

  virtual void setPropertiesFromCamera( const Ogre::Camera *camera );

  rviz::RenderPanel *render_panel_ = nullptr;
  Ogre::SceneNode *target_scene_node_ = nullptr;
  rviz::Shape *focal_shape_ = nullptr;

  /* ROS Interface */
  ros::NodeHandle update_nh_;
  ros::Publisher tracked_frame_pub_;
  ros::Publisher view_mode_pub_;
  ros::ServiceServer move_eye_service_;
  ros::ServiceServer move_eye_and_focus_service_;
  ros::ServiceServer set_view_mode_service_;
  ros::ServiceServer track_frame_service_;

  /* Animation & Frame Tracking */
  std::unique_ptr<CameraAnimator> camera_animator_;

  /* RViz Properties */
  /* 3D */
  rviz::FloatProperty *distance_property_ = nullptr;
  rviz::VectorProperty *focus_point_property_ = nullptr;
  rviz::VectorProperty *eye_point_property_ = nullptr;
  rviz::VectorProperty *up_vector_property_ = nullptr;
  rviz::VectorProperty *camera3d_offset_ = nullptr;
  /* 2D */
  rviz::FloatProperty *angle_property_ = nullptr;

  rviz::FloatProperty *animation_duration_property_ = nullptr;
  rviz::TfFrameProperty *tracked_frame_property_ = nullptr;
  rviz::FloatProperty *tracked_frame_p_gain_property_ = nullptr;

  rviz::BoolProperty *mode2d_property_ = nullptr;

  rviz::BoolProperty *keyboard_navigation_property_ = nullptr;
  rviz::FloatProperty *max_movement_property_ = nullptr;

  rviz::BoolProperty *enable_topics_property_ = nullptr;
  rviz::BoolProperty *enable_services_property_ = nullptr;

  /* Movement */
  int key_x_direction_ = 0;
  int key_y_direction_ = 0;

  /* Flags */
  ViewMode mode_ = view_modes::Mode3D;
  bool position_properties_connected_ = false;
  bool quick_mode_ = false;
  bool dragging_ = false;
  bool in_mode_transition_ = false;

};
}
#endif //HECTOR_RVIZ_PLUGINS_HECTOR_VIEW_CONTROLLER_H
