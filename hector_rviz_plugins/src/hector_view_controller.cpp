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

#include "hector_rviz_plugins/hector_view_controller.hpp"

#include "./view_controller_detail/camera_controller.hpp"
#include "./view_controller_detail/view_controller_ros_interface.hpp"

#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/frame_position_tracking_view_controller.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/geometry.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/orthographic.hpp>

#include <rviz_default_plugins/view_controllers/ortho/fixed_orientation_ortho_view_controller.hpp>

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreViewport.h>

#include <std_msgs/msg/string.hpp>

#include <QKeyEvent>

#include <hector_rviz_plugins_msgs/srv/move_eye.h>
#include <hector_rviz_plugins_msgs/srv/move_eye_and_focus.hpp>
#include <hector_rviz_plugins_msgs/srv/set_view_mode.hpp>
#include <hector_rviz_plugins_msgs/srv/track_frame.hpp>

namespace hector_rviz_plugins
{
namespace
{
constexpr float ORTHO_VIEW_CONTROLLER_CAMERA_Z = 500;
constexpr float DISTANCE_SCALE_FACTOR = 2200;

Ogre::SceneNode *getCameraParent( Ogre::Camera *camera )
{
  auto result = camera->getParentSceneNode();
  if ( result == nullptr ) {
    throw std::runtime_error( "Camera has no parent scene node." );
  }
  return result;
}

const Ogre::SceneNode *getCameraParent( const Ogre::Camera *camera )
{
  auto result = camera->getParentSceneNode();
  if ( result == nullptr ) {
    throw std::runtime_error( "Camera has no parent scene node." );
  }
  return result;
}

Ogre::Vector3 getCameraPosition( const Ogre::Camera *camera )
{
  auto parent = getCameraParent( camera );
  return parent->getPosition();
}

void setCameraPosition( Ogre::Camera *camera, const Ogre::Vector3 &position )
{
  auto parent = getCameraParent( camera );
  parent->setPosition( position );
}

void setCameraFixedYawAxis( Ogre::Camera *camera, const Ogre::Vector3 &fixed_axis )
{
  auto parent = getCameraParent( camera );
  parent->setFixedYawAxis( true, fixed_axis );
}

void setCameraDirection( Ogre::Camera *camera, const Ogre::Vector3 &direction )
{
  auto parent = getCameraParent( camera );
  parent->setDirection( direction, Ogre::Node::TS_PARENT );
}

Ogre::Quaternion getCameraOrientation( const Ogre::Camera *camera )
{
  auto parent = getCameraParent( camera );
  return parent->getOrientation();
}

void setCameraOrientation( Ogre::Camera *camera, const Ogre::Quaternion &orientation )
{
  auto parent = getCameraParent( camera );
  parent->setOrientation( orientation );
}
} // namespace

HectorViewController::HectorViewController()
{
  using namespace rviz_common::properties;
  up_vector_property_ = new VectorProperty( "Up", Ogre::Vector3::UNIT_Z,
                                            "The vector marking the up direction.", this );
  focus_point_property_ =
      new VectorProperty( "Focus", Ogre::Vector3::ZERO, "The focus point position.", this,
                          SLOT( onFocusPointPropertyChanged() ), this );
  eye_point_property_ = new VectorProperty( "Eye", Ogre::Vector3( 4, 0, 3 ), "The camera position.",
                                            this, SLOT( onEyePropertyChanged() ), this );
  distance_property_ = new FloatProperty( "Distance", 5.0, "The distance from camera to focus point.",
                                          this, SLOT( onDistancePropertyChanged() ), this );
  distance_property_->setMin( 0.01f );
  angle_property_ = new FloatProperty( "Angle", 0, "", this );

  animation_duration_property_ = new FloatProperty(
      "Animation Duration", 1,
      "The time the view controller takes to animate to the goal position in seconds.", this,
      SLOT( onAnimationDurationChanged() ), this );
  mode2d_property_ = new BoolProperty(
      "2D Mode",
      false, "If activated the view switches to a top down 2D mode similar to the TopDownOrtho viewcontroller. (Ctrl+D)",
      this, SLOT( onMode2DChanged() ), this );

  keyboard_navigation_property_ = new BoolProperty(
      "Keyboard Navigation", true, "If checked, the arrow keys can be used to navigate the camera.",
      this, SLOT( onKeyboardNavigationChanged() ), this );
  max_movement_property_ = new FloatProperty( "Movement speed (m/s)", 4, "The maximum speed.", this );

  enable_topics_property_ = new BoolProperty(
      "Enable Topics", true, "If checked, publishes information about the camera configuration.",
      this, SLOT( onEnableTopicsChanged() ), this );

  enable_services_property_ = new BoolProperty(
      "Enable Services", true,
      "If checked, allows external ROS nodes to modify your camera using provided services.", this,
      SLOT( onEnableServicesChanged() ), this );
}

HectorViewController::~HectorViewController()
{
  context_->getSceneManager()->destroySceneNode( target_scene_node_ );
}

void HectorViewController::lookAt( const Ogre::Vector3 &point )
{
  focus_point_property_->setVector( point );
}

void HectorViewController::mimic( rviz_common::ViewController *source_view )
{
  rviz_common::ViewController::mimic( source_view );
  if ( source_view->getClassId() == "rviz_default_plugins/TopDownOrtho" ) {
    setMode( view_modes::Mode2D, false );
    if ( source_view->getFocalPointStatus().exists_ ) {
      setFocusPoint( source_view->getFocalPointStatus().value_ );
    } else {
      Ogre::Vector3 pos = getCameraPosition( source_view->getCamera() );
      pos.z = 0;
      setFocusPoint( pos );
    }
  } else {
    setEyePoint( getCameraPosition( source_view->getCamera() ) );
    if ( source_view->getFocalPointStatus().exists_ ) {
      setFocusPoint( source_view->getFocalPointStatus().value_ );
    } else {
      // if the previous view does not have a focal point, we determine it from the orientation and
      // use a fixed distance.
      setFocusPoint( eye_point_property_->getVector() +
                     getCameraOrientation( source_view->getCamera() ) * Ogre::Vector3 ::UNIT_Z * 3 );
    }
    setCameraFixedYawAxis( camera_, Ogre::Vector3::UNIT_Z );
    updateDistance();
  }
}

bool HectorViewController::isTrackingFrame() const { return camera_animator_->isFrameTracked(); }

std::string HectorViewController::trackedFrame() const { return camera_animator_->trackedFrame(); }

void HectorViewController::onInitialize()
{
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  ros_interface_ =
      std::make_shared<ViewControllerRosInterface>( *this, node, context_->getFrameManager() );

  camera_animator_ = std::make_unique<CameraAnimator>( context_ );
  camera_animator_->setAnimationDuration( animation_duration_property_->getFloat() );
  tracked_frame_property_ = new rviz_common::properties::TfFrameProperty(
      "Tracked Frame",
      "", "The tracked frame. The view controller will follow this frame as it changes. Leave empty to disable tracking.",
      this, nullptr, false, SLOT( onTrackedFrameChanged() ), this );
  tracked_frame_p_gain_property_ = new rviz_common::properties::FloatProperty(
      "Tracked Frame P-Gain", camera_animator_->pGain(),
      "The P-Gain used to follow the tracked frame. This parameter is scaled by delta t.", this,
      SLOT( onPGainChanged() ), this );

  camera_->setCustomProjectionMatrix( false, Ogre::Matrix4::IDENTITY );
  camera_->setProjectionType( Ogre::PT_PERSPECTIVE );
  getCameraParent( camera_ )->setFixedYawAxis( true, Ogre::Vector3::UNIT_Z );

  render_panel_ = context_->getViewManager()->getRenderPanel();
  max_movement_property_->setHidden( !keyboard_navigation_property_->getBool() );
  if ( keyboard_navigation_property_->getBool() ) {
    render_panel_->installEventFilter( this );
  }

  target_scene_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  camera_->detachFromParent();
  target_scene_node_->attachObject( camera_ );
  focal_shape_ = std::make_unique<rviz_rendering::Shape>(
      rviz_rendering::Shape::Sphere, context_->getSceneManager(), target_scene_node_ );
  focal_shape_->setColor( 1.0f, 1.0f, 0.0f, 0.5f );
  focal_shape_->setScale( Ogre::Vector3( 0.1f, 0.1f, 0.1f ) );
  focal_shape_->getRootNode()->setVisible( false );

  onEnableTopicsChanged();
  onEnableServicesChanged();
}

void HectorViewController::reset()
{
  stopTracking();
  cancelAnimation();
  mode2d_property_->setBool( false );
  setFocusPoint( Ogre::Vector3::ZERO );
  setEyePoint( Ogre::Vector3( 4, 0, 3 ) );

  setFocusPoint( Ogre::Vector3::ZERO );
  setEyePoint( Ogre::Vector3( 4, 0, 3 ) );
  up_vector_property_->setVector( Ogre::Vector3::UNIT_Z );
  updateDistance();
}

void HectorViewController::onEnableTopicsChanged()
{
  if ( enable_topics_property_->getBool() ) {
    ros_interface_->enableTopics();
  } else {
    ros_interface_->disableTopics();
  }
}

void HectorViewController::onEnableServicesChanged()
{
  if ( enable_services_property_->getBool() ) {
    ros_interface_->enableServices();
  } else {
    ros_interface_->disableServices();
  }
}

void HectorViewController::handleKeyEvent( QKeyEvent *event, rviz_common::RenderPanel * )
{
  if ( ( event->modifiers() & Qt::ControlModifier ) == 0 )
    return;
  if ( event->key() == Qt::Key_D ) {
    mode2d_property_->setBool( !mode2d_property_->getBool() );
  }
}

bool HectorViewController::eventFilter( QObject *, QEvent *event )
{
  if ( event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease ) {
    bool move = event->type() == QEvent::KeyPress && render_panel_->hasFocus();
    auto *key_event = dynamic_cast<const QKeyEvent *>( event );
    quick_mode_ = ( key_event->modifiers() & Qt::ShiftModifier ) != 0;
    switch ( key_event->key() ) {
    case Qt::Key_Left:
      key_x_direction_ = move ? -1 : 0;
      break;
    case Qt::Key_Up:
      key_y_direction_ = move ? 1 : 0;
      break;
    case Qt::Key_Right:
      key_x_direction_ = move ? 1 : 0;
      break;
    case Qt::Key_Down:
      key_y_direction_ = move ? -1 : 0;
      break;
    default:
      break;
    }
  }
  return false;
}

void HectorViewController::moveOnXYPlaneBy( float dx, float dy )
{
  stopTracking();
  cancelAnimation();
  Ogre::Vector3 update( dx, dy, 0 );

  eye_point_property_->setVector( eye_point_property_->getVector() + update );
  focus_point_property_->setVector( focus_point_property_->getVector() + update );
  context_->queueRender();
}

void HectorViewController::moveEyeWithFocusTo( const Ogre::Vector3 &eye, bool stop_tracking,
                                               bool animate, bool switch_to_3d_mode )
{
  moveEyeWithNewFocus(
      eye, focus_point_property_->getVector() + ( eye - eye_point_property_->getVector() ),
      stop_tracking, animate, switch_to_3d_mode );
}

void HectorViewController::zoom( float ddistance ) { distance_property_->add( -ddistance ); }

void HectorViewController::cancelAnimation() { camera_animator_->stop(); }

void HectorViewController::moveEyeWithNewFocus( const Ogre::Vector3 &eye,
                                                const Ogre::Vector3 &focus, bool stop_tracking,
                                                bool animate, bool switch_to_3d_mode )
{
  HECTOR_RVIZ_LOG_DEBUG_STREAM( "Moving eye to (" << eye << ") and focus to (" << focus << ")" );

  cancelAnimation();
  if ( stop_tracking ) {
    stopTracking();
  }
  if ( !in_mode_transition_ && switch_to_3d_mode ) {
    setMode( view_modes::Mode3D );
  }
  if ( animate ) {
    camera_animator_->animate( eye_point_property_->getVector(), eye,
                               focus_point_property_->getVector(), focus );
  } else {
    setEyePoint( eye );
    setFocusPoint( focus );
    context_->queueRender();
  }
}
void HectorViewController::orbitEye(double yaw_delta, double theta_delta, bool stop_tracking,
                                                bool animate, bool switch_to_3d_mode)
{
  // Get current eye & focus points
  const Ogre::Vector3 eye = eye_point_property_->getVector();
  const Ogre::Vector3 focus = focus_point_property_->getVector();

  // Vector from focus → eye
  Ogre::Vector3 dir = eye - focus;
  const float  radius = dir.length();
  if (radius < 1e-6f) {
    HECTOR_RVIZ_LOG_WARN_STREAM( "orbitEye: eye and focus are too close together");
    return;
  }

  // Compute current spherical angles
  const float yaw = std::atan2( dir.y, dir.x );
  const float theta = std::atan2(dir.z, std::sqrt(dir.x*dir.x + dir.y*dir.y));

  // Apply deltas
  const float yaw_new   = yaw + static_cast<float>(yaw_delta);
  float theta_new = theta + static_cast<float>(theta_delta);

  // Clamp theta to avoid flipping
  const float max_theta = Ogre::Math::HALF_PI - 0.01f;
  theta_new = Ogre::Math::Clamp(theta_new, -max_theta, max_theta);

  // Reconstruct new direction vector in Cartesian coords
  const float cos_theta = std::cos(theta_new);
  Ogre::Vector3 new_dir{
    cos_theta * std::cos(yaw_new),  // x
    cos_theta * std::sin(yaw_new),  // y
    std::sin(theta_new)             // z
  };
  new_dir *= radius;  // scale back to original distance

  // Compute new eye position
  const Ogre::Vector3 new_eye = focus + new_dir;
  moveEyeWithNewFocus( new_eye, focus, stop_tracking, animate, switch_to_3d_mode );
}

ViewMode HectorViewController::mode() const { return mode_; }

void HectorViewController::setMode( ViewMode value, bool animate_transition )
{
  if ( mode_ == value )
    return;
  mode_ = value;

  Ogre::Vector3 focus = focus_point_property_->getVector();
  if ( value == view_modes::Mode2D ) {
    in_mode_transition_ = true;
    moveEyeWithNewFocus( focus + Ogre::Vector3::UNIT_Z * distance_property_->getFloat() +
                             Ogre::Vector3::UNIT_X / 10,
                         focus, false, animate_transition );
  } else {
    camera_->setCustomProjectionMatrix( false, Ogre::Matrix4::IDENTITY );
    camera_->setProjectionType( Ogre::PT_PERSPECTIVE );
    getCameraParent( camera_ )->setFixedYawAxis( true, Ogre::Vector3::UNIT_Z );
    moveEyeWithNewFocus( eye_point_property_->getVector(), focus, false, animate_transition );
  }
  ros_interface_->publishViewMode();
  emit modeChanged();
}

void HectorViewController::trackFrame( const std::string &name )
{
  if ( camera_animator_->trackedFrame() == name )
    return;
  camera_animator_->trackFrame( name );
  tracked_frame_property_->setStdString( name );
  ros_interface_->publishTrackedFrame();
  emit trackingChanged( camera_animator_->isFrameTracked(), camera_animator_->trackedFrame() );
}

void HectorViewController::stopTracking()
{
  if ( !camera_animator_->isFrameTracked() )
    return;
  camera_animator_->stopTracking();
  tracked_frame_property_->setString( "" );
  ros_interface_->publishTrackedFrame();
  emit trackingChanged( camera_animator_->isFrameTracked(), camera_animator_->trackedFrame() );
}

void HectorViewController::onMode2DChanged()
{
  setMode( mode2d_property_->getBool() ? view_modes::Mode2D : view_modes::Mode3D );
  angle_property_->setHidden( !mode2d_property_->getBool() );
  eye_point_property_->setHidden( mode2d_property_->getBool() );
}

void HectorViewController::onAnimationDurationChanged()
{
  camera_animator_->setAnimationDuration( animation_duration_property_->getFloat() );
}

void HectorViewController::onEyePropertyChanged() { updateDistance(); }

void HectorViewController::onFocusPointPropertyChanged() { updateDistance(); }

void HectorViewController::onDistancePropertyChanged()
{
  setEyePoint( focus_point_property_->getVector() +
               distance_property_->getFloat() * getCameraOrientation( camera_ ).zAxis() );
}

void HectorViewController::onTrackedFrameChanged()
{
  trackFrame( tracked_frame_property_->getFrameStd() );
}

void HectorViewController::onPGainChanged()
{
  camera_animator_->setPGain( tracked_frame_p_gain_property_->getFloat() );
}

void HectorViewController::onKeyboardNavigationChanged()
{
  max_movement_property_->setHidden( !keyboard_navigation_property_->getBool() );
  if ( keyboard_navigation_property_->getBool() ) {
    render_panel_->installEventFilter( this );
  } else {
    render_panel_->removeEventFilter( this );
  }
}

void HectorViewController::updateOrientation( float delta_yaw, float delta_pitch, float delta_roll )
{
  Ogre::Quaternion old_camera_orientation = getCameraOrientation( camera_ );

  Ogre::Quaternion yaw( Ogre::Radian( delta_yaw ), Ogre::Vector3::UNIT_Y );
  Ogre::Quaternion pitch( Ogre::Radian( delta_pitch ), Ogre::Vector3::UNIT_X );
  Ogre::Quaternion roll( Ogre::Radian( delta_roll ), Ogre::Vector3::UNIT_Z );
  Ogre::Quaternion orientation_change = yaw * pitch * roll;
  Ogre::Quaternion new_camera_orientation = old_camera_orientation * orientation_change;

  Ogre::Vector3 focus = focus_point_property_->getVector();
  Ogre::Vector3 new_eye_position =
      focus + distance_property_->getFloat() * new_camera_orientation.zAxis();
  setEyePoint( new_eye_position );
  setCameraPosition( camera_, new_eye_position );
  setCameraFixedYawAxis( camera_, Ogre::Vector3::UNIT_Z );
  setCameraDirection( camera_, focus - new_eye_position );
}

void HectorViewController::handleMouseEvent( rviz_common::ViewportMouseEvent &evt )
{
  if ( mode() == view_modes::Mode3D ) {
    handleMouseEvent3D( evt );
  } else {
    handleMouseEvent2D( evt );
  }
}

void HectorViewController::handleMouseEvent2D( rviz_common::ViewportMouseEvent &evt )
{
  setStatus( "<b>Left-Click:</b> Rotate. <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b> Zoom. "
             "<b>Ctrl+D:</b> Switch to 3D. <b>Shift+Left-Click:</b> Move X/Y." );

  float diff_x = 0;
  float diff_y = 0;
  bool changed = false;

  if ( evt.type == QEvent::MouseButtonPress ) {
    dragging_ = true;
    focal_shape_->getRootNode()->setVisible( true );
  } else if ( evt.type == QEvent::MouseButtonRelease ) {
    focal_shape_->getRootNode()->setVisible( false );
    dragging_ = false;
  } else if ( dragging_ && evt.type == QEvent::MouseMove ) {
    diff_x = static_cast<float>( evt.x - evt.last_x ) / static_cast<float>( evt.device_pixel_ratio );
    diff_y = static_cast<float>( evt.y - evt.last_y ) / static_cast<float>( evt.device_pixel_ratio );
    changed = true;
  }

  if ( evt.left() && !evt.shift() ) {
    setCursor( Rotate2D );
    angle_property_->add( diff_x * 0.005f );
    setCameraOrientation( camera_, Ogre::Quaternion( Ogre::Radian( angle_property_->getFloat() ),
                                                     Ogre::Vector3::UNIT_Z ) );
  } else if ( evt.middle() || evt.left() ) {
    setCursor( MoveXY );
    float cosa = std::cos( angle_property_->getFloat() );
    float sina = std::sin( angle_property_->getFloat() );
    moveOnXYPlaneBy(
        ( -diff_x * cosa - diff_y * sina ) * distance_property_->getFloat() / DISTANCE_SCALE_FACTOR,
        ( -diff_x * sina + diff_y * cosa ) * distance_property_->getFloat() / DISTANCE_SCALE_FACTOR );
  } else if ( evt.right() ) {
    setCursor( Zoom );
    distance_property_->multiply( 1.0f + diff_y * 0.01f );
  } else {
    setCursor( evt.shift() ? MoveXY : Rotate2D );
  }

  if ( evt.wheel_delta != 0 ) {
    distance_property_->multiply( 1.0f - static_cast<float>( evt.wheel_delta ) * 0.001f );
    changed = true;
  }

  if ( changed )
    context_->queueRender();
}

void HectorViewController::handleMouseEvent3D( rviz_common::ViewportMouseEvent &evt )
{
  setStatus( "<b>Left-Click:</b>  Rotate. <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b> "
             "Zoom.  <b>Shift</b>: Move faster. <b>Ctrl+D:</b> Switch to 2D." );

  float distance = distance_property_->getFloat();
  float diff_x = 0;
  float diff_y = 0;

  if ( evt.wheel_delta != 0 ) {
    zoom( static_cast<float>( evt.wheel_delta ) * 0.001f * distance );
  }

  if ( evt.type == QEvent::MouseButtonPress ) {
    dragging_ = true;
    focal_shape_->getRootNode()->setVisible( true );
  } else if ( evt.type == QEvent::MouseButtonRelease ) {
    focal_shape_->getRootNode()->setVisible( false );
    dragging_ = false;
  } else if ( dragging_ && evt.type == QEvent::MouseMove ) {
    diff_x = static_cast<float>( evt.x - evt.last_x ) / static_cast<float>( evt.device_pixel_ratio );
    diff_y = static_cast<float>( evt.y - evt.last_y ) / static_cast<float>( evt.device_pixel_ratio );
  }

  if ( evt.left() && !evt.shift() ) {
    setCursor( Rotate3D );
    updateOrientation( -diff_x * 0.005f, -diff_y * 0.005f, 0 );
  } else if ( evt.middle() || ( evt.shift() && evt.left() ) ) {
    setCursor( MoveXY );
    float fov_y = camera_->getFOVy().valueRadians();
    float fov_x = 2.f * atanf( tanf( fov_y / 2 ) * camera_->getAspectRatio() );

    int width = camera_->getViewport()->getActualWidth();
    int height = camera_->getViewport()->getActualWidth();

    Ogre::Vector3 eye = eye_point_property_->getVector();
    Ogre::Vector3 translation;
    translation.x =
        diff_x == 0 ? 0 : -( diff_x / (float)width ) * distance * tanf( fov_x / 2.0f ) * 2.0f;
    translation.y =
        diff_y == 0 ? 0 : ( diff_y / (float)height ) * distance * tanf( fov_y / 2.0f ) * 2.0f;
    translation.z = 0;
    moveEyeWithFocusTo( eye + getCameraParent( camera_ )->getOrientation() * translation, true,
                        false );
  } else if ( evt.right() ) {
    if ( evt.shift() ) {
      setCursor( MoveZ );
      Ogre::Vector3 eye = eye_point_property_->getVector();
      auto translation = Ogre::Vector3( 0, 0, diff_y * 0.1f * distance / 10.0f );
      moveEyeWithFocusTo( eye + getCameraParent( camera_ )->getOrientation() * translation, true,
                          false );
    } else {
      setCursor( Zoom );
      zoom( -diff_y * 0.1f * distance / 10.0f );
    }
  }
}

void HectorViewController::update( float dt, float )
{
  updateMovement( dt );
  updateCamera( dt );
}

void HectorViewController::updateMovement( float dt )
{

  if ( key_x_direction_ == 0 && key_y_direction_ == 0 )
    return;

  auto x = static_cast<float>( key_x_direction_ );
  auto y = static_cast<float>( key_y_direction_ );
  float length = std::abs( x ) + std::abs( y );
  float x_ratio = x / length;
  float y_ratio = y / length;

  x = x_ratio * max_movement_property_->getFloat() * ( dt * 1E-9f );
  y = y_ratio * max_movement_property_->getFloat() * ( dt * 1E-9f );

  if ( quick_mode_ ) {
    x *= 2;
    y *= 2;
  }

  // This is y direction because we treat mouse up as y
  Ogre::Vector3 y_direction = eye_point_property_->getVector() - focus_point_property_->getVector();
  y_direction.z = 0;
  y_direction.normalise();
  Ogre::Vector3 x_direction = y_direction.crossProduct( up_vector_property_->getVector() );
  x_direction.z = 0;
  x_direction.normalise();
  moveOnXYPlaneBy( -x * x_direction.x - y * y_direction.x, -x * x_direction.y - y * y_direction.y );
}

void HectorViewController::updateCamera( float dt )
{
  Ogre::Vector3 focus = focus_point_property_->getVector();
  Ogre::Vector3 eye = eye_point_property_->getVector();
  float distance = distance_property_->getFloat();

  bool camera_position_changed = camera_animator_->updateCamera( eye, focus, dt * 1E-9f );
  focal_shape_->setPosition( focus_point_property_->getVector() );

  if ( camera_position_changed && !in_mode_transition_ ) {
    setEyePoint( eye );
    setFocusPoint( focus );
    updateDistance();
  }

  if ( mode() == view_modes::Mode2D && !camera_position_changed ) {
    if ( in_mode_transition_ ) {
      camera_->setProjectionType( Ogre::PT_ORTHOGRAPHIC );
      in_mode_transition_ = false;
    }
    auto width = static_cast<float>( camera_->getViewport()->getActualWidth() );
    auto height = static_cast<float>( camera_->getViewport()->getActualHeight() );
    float scale = DISTANCE_SCALE_FACTOR / distance;
    Ogre::Matrix4 proj = rviz_rendering::buildScaledOrthoMatrix(
        -width / scale / 2, width / scale / 2, -height / scale / 2, height / scale / 2,
        camera_->getNearClipDistance(), camera_->getFarClipDistance() );
    camera_->setCustomProjectionMatrix( true, proj );
  }

  if ( mode() == view_modes::Mode2D && !in_mode_transition_ ) {
    setCameraPosition( camera_, Ogre::Vector3( focus.x, focus.y, ORTHO_VIEW_CONTROLLER_CAMERA_Z ) );
    setCameraOrientation( camera_, Ogre::Quaternion( Ogre::Radian( angle_property_->getFloat() ),
                                                     Ogre::Vector3::UNIT_Z ) );
  } else {
    setCameraPosition( camera_, eye );
    setCameraDirection( camera_, focus - eye );
  }
}

void HectorViewController::setEyePoint( const Ogre::Vector3 &eye )
{
  disconnect( eye_point_property_, SIGNAL( changed() ), this, SLOT( onEyePropertyChanged() ) );
  eye_point_property_->setVector( eye );
  connect( eye_point_property_, SIGNAL( changed() ), this, SLOT( onEyePropertyChanged() ) );
}

void HectorViewController::setFocusPoint( const Ogre::Vector3 &focus )
{
  disconnect( focus_point_property_, SIGNAL( changed() ), this,
              SLOT( onFocusPointPropertyChanged() ) );
  focus_point_property_->setVector( focus );
  connect( focus_point_property_, SIGNAL( changed() ), this, SLOT( onFocusPointPropertyChanged() ) );
}

void HectorViewController::updateDistance()
{
  setDistance( ( focus_point_property_->getVector() - eye_point_property_->getVector() ).length() );
}

void HectorViewController::setDistance( float distance )
{
  disconnect( distance_property_, SIGNAL( changed() ), this, SLOT( onDistancePropertyChanged() ) );
  distance_property_->setFloat( distance );
  connect( distance_property_, SIGNAL( changed() ), this, SLOT( onDistancePropertyChanged() ) );
}

} // namespace hector_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( hector_rviz_plugins::HectorViewController, rviz_common::ViewController )
