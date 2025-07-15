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

#ifndef HECTOR_RVIZ_PLUGINS_HECTOR_VIEW_CONTROLLER_HPP
#define HECTOR_RVIZ_PLUGINS_HECTOR_VIEW_CONTROLLER_HPP

#include <rviz_common/view_controller.hpp>

namespace rviz_rendering
{
class Shape;
} // namespace rviz_rendering

namespace rviz_common::properties
{
class TfFrameProperty;

class VectorProperty;
} // namespace rviz_common::properties

namespace hector_rviz_plugins
{

class CameraAnimator;

namespace view_modes
{
enum ViewMode { Mode3D = 0, Mode2D = 1 };
}
using ViewMode = view_modes::ViewMode;
class ViewControllerRosInterface;

class HectorViewController : public rviz_common::ViewController
{
  Q_OBJECT
public:
  HectorViewController();

  ~HectorViewController() override;

  void lookAt( const Ogre::Vector3 &point ) override;

  void mimic( rviz_common::ViewController *source_view ) override;

  void onInitialize() override;

  void reset() override;

  void handleMouseEvent( rviz_common::ViewportMouseEvent &evt ) override;

  void handleKeyEvent( QKeyEvent *event, rviz_common::RenderPanel *panel ) override;

  bool eventFilter( QObject *obj, QEvent *event ) override;

  void moveOnXYPlaneBy( float dx, float dy );

  void moveEyeWithFocusTo( const Ogre::Vector3 &eye, bool stop_tracking = true, bool animate = true,
                           bool switch_to_3d_mode = false );

  void moveEyeWithNewFocus( const Ogre::Vector3 &eye, const Ogre::Vector3 &focus,
                            bool stop_tracking = true, bool animate = true,
                            bool switch_to_3d_mode = true );
  void orbitEye( double yaw_delta, double theta_delta, bool stop_tracking, bool animate,
                 bool switch_to_3d_mode );
  void cancelAnimation();

  void zoom( float ddistance );

  ViewMode mode() const;

  void setMode( ViewMode value, bool animate_transition = true );

  void trackFrame( const std::string &name );

  bool isTrackingFrame() const;

  std::string trackedFrame() const;

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
  virtual void handleMouseEvent3D( rviz_common::ViewportMouseEvent &evt );

  virtual void handleMouseEvent2D( rviz_common::ViewportMouseEvent &evt );

  void update( float dt, float ros_dt ) override;

  void updateMovement( float dt );

  void updateCamera( float dt );

  void setEyePoint( const Ogre::Vector3 &eye );

  void setFocusPoint( const Ogre::Vector3 &focus );

  void updateDistance();

  void setDistance( float distance );

  virtual void updateOrientation( float delta_yaw, float delta_pitch, float delta_roll );

  rviz_common::RenderPanel *render_panel_ = nullptr;
  Ogre::SceneNode *target_scene_node_ = nullptr;
  std::unique_ptr<rviz_rendering::Shape> focal_shape_;

  std::shared_ptr<ViewControllerRosInterface> ros_interface_;

  /* Animation & Frame Tracking */
  std::unique_ptr<CameraAnimator> camera_animator_;

  /* RViz Properties */
  /* 3D */
  rviz_common::properties::FloatProperty *distance_property_ = nullptr;
  rviz_common::properties::VectorProperty *focus_point_property_ = nullptr;
  rviz_common::properties::VectorProperty *eye_point_property_ = nullptr;
  rviz_common::properties::VectorProperty *up_vector_property_ = nullptr;
  /* 2D */
  rviz_common::properties::FloatProperty *angle_property_ = nullptr;

  rviz_common::properties::FloatProperty *animation_duration_property_ = nullptr;
  rviz_common::properties::TfFrameProperty *tracked_frame_property_ = nullptr;
  rviz_common::properties::FloatProperty *tracked_frame_p_gain_property_ = nullptr;

  rviz_common::properties::BoolProperty *mode2d_property_ = nullptr;

  rviz_common::properties::BoolProperty *keyboard_navigation_property_ = nullptr;
  rviz_common::properties::FloatProperty *max_movement_property_ = nullptr;

  rviz_common::properties::BoolProperty *enable_topics_property_ = nullptr;
  rviz_common::properties::BoolProperty *enable_services_property_ = nullptr;

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
} // namespace hector_rviz_plugins
#endif // HECTOR_RVIZ_PLUGINS_HECTOR_VIEW_CONTROLLER_HPP
