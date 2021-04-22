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

#include "hector_rviz_plugins/hector_view_controller.h"

#include <rviz/frame_position_tracking_view_controller.h>
#include <rviz/ogre_helpers/orthographic.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/geometry.h>
#include <rviz/render_panel.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/view_manager.h>
#include <rviz/viewport_mouse_event.h>

#include <std_msgs/String.h>

#include <hector_rviz_plugins_msgs/MoveEye.h>
#include <hector_rviz_plugins_msgs/MoveEyeAndFocus.h>
#include <hector_rviz_plugins_msgs/TrackFrame.h>
#include <hector_rviz_plugins_msgs/ViewMode.h>

namespace hector_rviz_plugins
{
namespace
{
constexpr float DISTANCE_SCALE_FACTOR = 2200;
}

HectorViewController::HectorViewController() : update_nh_( "~" )
{
  up_vector_property_ = new rviz::VectorProperty( "Up", Ogre::Vector3::UNIT_Z, "The vector marking the up direction.",
                                                  this );
  focus_point_property_ = new rviz::VectorProperty( "Focus", Ogre::Vector3::ZERO, "The focus point position.", this );
  eye_point_property_ = new rviz::VectorProperty( "Eye", Ogre::Vector3( 4, 0, 3 ), "The camera position.", this );
  distance_property_ = new rviz::FloatProperty( "Distance", 5.0, "The distance from camera to focus point.",
                                                this );
  distance_property_->setMin( 0.01 );
  camera3d_offset_ = new rviz::VectorProperty( "Eye 3D Offset", Ogre::Vector3( 4, 0, 3 ),
                                               "The offset of the camera to the focal point when switching back to 3D mode.",
                                               this );
  camera3d_offset_->setHidden( true );
  angle_property_ = new rviz::FloatProperty( "Angle", 0, "", this );

  animation_duration_property_ = new rviz::FloatProperty( "Animation Duration", 1,
                                                          "The time the view controller takes to animate to the goal position in seconds.",
                                                          this );
  mode2d_property_ = new rviz::BoolProperty( "2D Mode", false,
                                             "If activated the view switches to a top down 2D mode similar to the TopDownOrtho viewcontroller. (Ctrl+D)",
                                             this, SLOT( onMode2DChanged()), this );

  keyboard_navigation_property_ = new rviz::BoolProperty( "Keyboard Navigation", true,
                                                          "If checked, the arrow keys can be used to navigate the camera.",
                                                          this, SLOT( onKeyboardNavigationChanged()), this );
  max_movement_property_ = new rviz::FloatProperty( "Movement speed (m/s)", 4, "The maximum speed.", this );

  enable_topics_property_ = new rviz::BoolProperty( "Enable Topics", true,
                                                    "If checked, publishes information about the camera configuration.",
                                                    this, SLOT( onEnableTopicsChanged()), this );

  enable_services_property_ = new rviz::BoolProperty( "Enable Services", true,
                                                      "If checked, allows external ROS nodes to modify your camera using provided services.",
                                                      this, SLOT( onEnableServicesChanged()), this );
}

HectorViewController::~HectorViewController()
{
  context_->getSceneManager()->destroySceneNode( target_scene_node_ );
  delete focal_shape_;
}

void HectorViewController::lookAt( const Ogre::Vector3 &point )
{
  focus_point_property_->setVector( point );
}

void HectorViewController::mimic( rviz::ViewController *source_view )
{
  Ogre::Camera *source_camera = source_view->getCamera();
  if ( source_view->getClassId() == "rviz/Orbit" )
  {
    distance_property_->setFloat( source_view->subProp( "Distance" )->getValue().toFloat());
  }
  else
  {
    distance_property_->setFloat( source_camera->getPosition().length());
  }

  setMode( source_view->getClassId() == "rviz/TopDownOrtho" ? view_modes::Mode2D : view_modes::Mode3D, false );

  setPropertiesFromCamera( source_camera );
}

void HectorViewController::onInitialize()
{
  tracked_frame_property_ = new rviz::TfFrameProperty( "Tracked Frame", "",
                                                       "The tracked frame. The view controller will follow this frame as it changes. Leave empty to disable tracking.",
                                                       this, context_->getFrameManager(), false,
                                                       SLOT( onTrackedFrameChanged()), this );
  tracked_frame_p_gain_property_ = new rviz::FloatProperty( "Tracked Frame P-Gain", 3,
                                                            "The P-Gain used to follow the tracked frame. This parameter is scaled by delta t.",
                                                            this );

  camera_->detachFromParent();
  camera_->setProjectionType( Ogre::PT_PERSPECTIVE );
  camera_->setFixedYawAxis( true, Ogre::Vector3::UNIT_Z );
  connectPositionProperties();

  render_panel_ = context_->getViewManager()->getRenderPanel();
  max_movement_property_->setHidden( !keyboard_navigation_property_->getBool());
  if ( keyboard_navigation_property_->getBool())
  {
    render_panel_->installEventFilter( this );
  }

  target_scene_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  target_scene_node_->attachObject( camera_ );
  focal_shape_ = new rviz::Shape( rviz::Shape::Sphere, context_->getSceneManager(), target_scene_node_ );
  focal_shape_->setColor( 1.0f, 1.0f, 0.0f, 0.5f );
  focal_shape_->setScale( Ogre::Vector3( 0.1, 0.1, 0.1 ));
  focal_shape_->getRootNode()->setVisible( false );

  update_nh_.setCallbackQueue( context_->getUpdateQueue());
  onEnableTopicsChanged();
  onEnableServicesChanged();
}

void HectorViewController::reset()
{
  stopTracking();
  cancelAnimation();
  mode2d_property_->setBool( false );
  disconnectPositionProperties();
  focus_point_property_->setVector( Ogre::Vector3( 0, 0, 0 ));
  eye_point_property_->setVector( Ogre::Vector3( 4, 0, 3 ));
  up_vector_property_->setVector( Ogre::Vector3::UNIT_Z );
  updateDistance();
  connectPositionProperties();
}

void HectorViewController::onEnableTopicsChanged()
{
  if ( enable_topics_property_->getBool())
  {
    tracked_frame_pub_ = update_nh_.advertise<std_msgs::String>( "hector_view_controller/tracked_frame", 1, true );
    view_mode_pub_ = update_nh_.advertise<hector_rviz_plugins_msgs::ViewMode>( "hector_view_controller/view_mode", 1,
                                                                               true );
    publishTrackedFrame();
    publishViewMode();
  }
  else
  {
    tracked_frame_pub_.shutdown();
    view_mode_pub_.shutdown();
  }
}

void HectorViewController::onEnableServicesChanged()
{
  using namespace hector_rviz_plugins_msgs;
  if ( enable_services_property_->getBool())
  {
    if ( !move_eye_service_ )
    {
      move_eye_service_ = update_nh_.advertiseService(
        "hector_view_controller/move_eye",
        boost::function<bool( MoveEyeRequest &, MoveEyeResponse & )>(
          [ this ]( MoveEyeRequest &req, MoveEyeResponse & )
          {
            if ( req.header.frame_id.empty()) req.header.frame_id = context_->getFrameManager()->getFixedFrame();
            geometry_msgs::Pose pose;
            pose.orientation.w = 1;
            pose.position = req.eye;
            Ogre::Vector3 pos;
            Ogre::Quaternion _;
            context_->getFrameManager()->transform( req.header, pose, pos, _ );
            moveEyeWithFocusTo( pos, req.stop_tracking, !req.disable_animation, req.switch_to_3d_mode );
            return true;
          } ));
    }
    if ( !move_eye_and_focus_service_ )
    {
      move_eye_and_focus_service_ = update_nh_.advertiseService(
        "hector_view_controller/move_eye_and_focus",
        boost::function<bool( MoveEyeAndFocusRequest &, MoveEyeAndFocusResponse & )>(
          [ this ]( MoveEyeAndFocusRequest &req, MoveEyeAndFocusResponse & )
          {
            if ( req.header.frame_id.empty()) req.header.frame_id = context_->getFrameManager()->getFixedFrame();
            geometry_msgs::Pose pose;
            pose.orientation.w = 1;
            pose.position = req.eye;
            Ogre::Vector3 eye;
            Ogre::Vector3 focus;
            Ogre::Quaternion _;
            context_->getFrameManager()->transform( req.header, pose, eye, _ );
            pose.position = req.focus;
            context_->getFrameManager()->transform( req.header, pose, focus, _ );
            moveEyeWithNewFocus( eye, focus, req.stop_tracking, !req.disable_animation );
            return true;
          } ));
    }
    if ( !set_view_mode_service_ )
    {
      set_view_mode_service_ = update_nh_.advertiseService(
        "hector_view_controller/set_view_mode",
        boost::function<bool( SetViewModeRequest &, SetViewModeResponse & )>(
          [ this ]( SetViewModeRequest &req, SetViewModeResponse & )
          {
            setMode( req.mode.mode == hector_rviz_plugins_msgs::ViewMode::MODE_3D
                     ? view_modes::Mode3D
                     : view_modes::Mode2D,
                     !req.disable_animation );
            return true;
          }
        ));
    }
    if ( !track_frame_service_ )
    {
      track_frame_service_ = update_nh_.advertiseService(
        "hector_view_controller/set_tracked_frame",
        boost::function<bool( TrackFrameRequest &, TrackFrameResponse & )>(
          [ this ]( TrackFrameRequest &req, TrackFrameResponse & )
          {
            trackFrame( req.frame );
            return true;
          } ));
    }
  }
  else
  {
    move_eye_service_.shutdown();
    move_eye_and_focus_service_.shutdown();
    set_view_mode_service_.shutdown();
    track_frame_service_.shutdown();
  }
}

void HectorViewController::handleMouseEvent( rviz::ViewportMouseEvent &evt )
{
  if ( mode() == view_modes::Mode3D )
  {
    handleMouseEvent3D( evt );
  }
  else
  {
    handleMouseEvent2D( evt );
  }
}

void HectorViewController::handleKeyEvent( QKeyEvent *event, rviz::RenderPanel * )
{
  if ((event->modifiers() & Qt::ControlModifier) == 0 ) return;
  if ( event->key() == Qt::Key_D )
  {
    mode2d_property_->setBool( !mode2d_property_->getBool());
  }
}

bool HectorViewController::eventFilter( QObject *, QEvent *event )
{
  if ( event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease )
  {
    bool move = event->type() == QEvent::KeyPress && render_panel_->hasFocus();
    auto *key_event = dynamic_cast<QKeyEvent *>(event);
    quick_mode_ = (key_event->modifiers() & Qt::ShiftModifier) != 0;
    switch ( key_event->key())
    {
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
    }
  }
  return false;
}

void HectorViewController::moveOnXYPlaneBy( float dx, float dy )
{
  stopTracking();
  cancelAnimation();
  Ogre::Vector3 update( dx, dy, 0 );

  disconnectPositionProperties();
  eye_point_property_->setVector( eye_point_property_->getVector() + update );
  focus_point_property_->setVector( focus_point_property_->getVector() + update );
  connectPositionProperties();
}

void HectorViewController::moveEyeWithFocusTo( const Ogre::Vector3 &eye, bool stop_tracking, bool animate,
                                               bool switch_to_3d_mode )
{
  moveEyeWithNewFocus( eye, focus_point_property_->getVector() + (eye - eye_point_property_->getVector()),
                       stop_tracking, animate, switch_to_3d_mode );
}

void HectorViewController::zoom( float ddistance )
{
  distance_property_->add( -ddistance );
}

void HectorViewController::cancelAnimation() { in_animation_ = false; }

void HectorViewController::moveEyeWithNewFocus( const Ogre::Vector3 &eye, const Ogre::Vector3 &focus,
                                                bool stop_tracking, bool animate, bool switch_to_3d_mode )
{
  ROS_DEBUG_STREAM_NAMED( "HectorViewController", "Moving eye to (" << eye << ") and focus to (" << focus << ")" );
  cancelAnimation();
  if ( stop_tracking )
  {
    stopTracking();
  }
  if ( !in_mode_transition_ && switch_to_3d_mode )
  {
    setMode( view_modes::Mode3D );
  }
  if ( animate )
  {
    // Reset tracked frame differences
    tracked_frame_remaining_position_difference_ = Ogre::Vector3::ZERO;
    tracked_frame_remaining_orientation_difference_ = Ogre::Quaternion::IDENTITY;

    animation_eye_start_ = eye_point_property_->getVector();
    animation_eye_goal_ = eye;

    animation_focus_start_ = focus_point_property_->getVector();
    animation_focus_goal_ = focus;

    animation_start_ = ros::WallTime::now();

    in_animation_ = true;
  }
  else
  {
    disconnectPositionProperties();
    eye_point_property_->setVector( eye );
    focus_point_property_->setVector( focus );
    connectPositionProperties();
  }
}

ViewMode HectorViewController::mode() const { return mode_; }

void HectorViewController::setMode( ViewMode value, bool animate_transition )
{
  if ( mode_ == value )
    return;
  mode_ = value;

  Ogre::Vector3 focus = focus_point_property_->getVector();
  if ( value == view_modes::Mode2D )
  {
    camera3d_offset_->setVector( eye_point_property_->getVector() - focus );
    in_mode_transition_ = true;
    moveEyeWithNewFocus( focus + Ogre::Vector3::UNIT_Z * distance_property_->getFloat() + Ogre::Vector3::UNIT_X / 10,
                         focus, false, animate_transition );
  }
  else
  {
    camera_->setCustomProjectionMatrix( false, Ogre::Matrix4::IDENTITY );
    camera_->setProjectionType( Ogre::PT_PERSPECTIVE );
    camera_->setFixedYawAxis( true, Ogre::Vector3::UNIT_Z );
    moveEyeWithNewFocus( focus + camera3d_offset_->getVector().normalisedCopy() * distance_property_->getFloat(), focus,
                         false, animate_transition );
  }
  publishViewMode();
  emit modeChanged();
}

void HectorViewController::trackFrame( const std::string &name )
{
  if ( tracked_frame_ == name ) return;
  tracked_frame_ = name;
  is_frame_tracked_ = !name.empty();
  if ( is_frame_tracked_ )
  {
    context_->getFrameManager()->getTransform( name, ros::Time(), tracked_frame_last_position_,
                                               tracked_frame_last_orientation_ );
    tracked_frame_remaining_position_difference_ = Ogre::Vector3::ZERO;
    tracked_frame_remaining_orientation_difference_ = Ogre::Quaternion::IDENTITY;
  }
  tracked_frame_property_->setStdString( name );
  publishTrackedFrame();
  emit trackingChanged( is_frame_tracked_, tracked_frame_ );
}

void HectorViewController::stopTracking()
{
  if ( !is_frame_tracked_ ) return;
  is_frame_tracked_ = false;
  tracked_frame_property_->setString( "" );
  publishTrackedFrame();
  emit trackingChanged( is_frame_tracked_, tracked_frame_ );
}

void HectorViewController::onMode2DChanged()
{
  setMode( mode2d_property_->getBool() ? view_modes::Mode2D : view_modes::Mode3D );
  angle_property_->setHidden( !mode2d_property_->getBool());
  eye_point_property_->setHidden( mode2d_property_->getBool());
}

void HectorViewController::onEyePropertyChanged()
{
  updateDistance();
}

void HectorViewController::onFocusPointPropertyChanged()
{
  updateDistance();
}

void HectorViewController::onDistancePropertyChanged()
{
  disconnectPositionProperties();
  eye_point_property_->setVector(
    focus_point_property_->getVector() + distance_property_->getFloat() * camera_->getOrientation().zAxis());
  connectPositionProperties();
}

void HectorViewController::onTrackedFrameChanged() { trackFrame( tracked_frame_property_->getFrameStd()); }

void HectorViewController::onKeyboardNavigationChanged()
{
  max_movement_property_->setHidden( !keyboard_navigation_property_->getBool());
  if ( keyboard_navigation_property_->getBool())
  {
    render_panel_->installEventFilter( this );
  }
  else
  {
    render_panel_->removeEventFilter( this );
  }
}

void HectorViewController::publishViewMode()
{
  if ( !view_mode_pub_ ) return;
  hector_rviz_plugins_msgs::ViewMode view_mode_msg;
  view_mode_msg.mode = mode_ == view_modes::Mode2D ? hector_rviz_plugins_msgs::ViewMode::MODE_2D
                                                   : hector_rviz_plugins_msgs::ViewMode::MODE_3D;
  view_mode_pub_.publish( view_mode_msg );
}

void HectorViewController::publishTrackedFrame()
{
  if ( !tracked_frame_pub_ ) return;
  std_msgs::String tracked_frame_msg;
  tracked_frame_msg.data = is_frame_tracked_ ? tracked_frame_ : "";
  tracked_frame_pub_.publish( tracked_frame_msg );
}

void HectorViewController::connectPositionProperties()
{
  position_properties_connected_ = true;
  connect( eye_point_property_, SIGNAL( changed()), this, SLOT( onEyePropertyChanged()));
  connect( focus_point_property_, SIGNAL( changed()), this, SLOT( onFocusPointPropertyChanged()));
  connect( distance_property_, SIGNAL( changed()), this, SLOT( onDistancePropertyChanged()));
}

void HectorViewController::disconnectPositionProperties()
{
  position_properties_connected_ = false;
  disconnect( eye_point_property_, SIGNAL( changed()), this, SLOT( onEyePropertyChanged()));
  disconnect( focus_point_property_, SIGNAL( changed()), this, SLOT( onFocusPointPropertyChanged()));
  disconnect( distance_property_, SIGNAL( changed()), this, SLOT( onDistancePropertyChanged()));
}

void HectorViewController::updateCamera()
{
  camera_->setPosition( eye_point_property_->getVector());
  camera_->setDirection( focus_point_property_->getVector() - eye_point_property_->getVector());

  focal_shape_->setPosition( focus_point_property_->getVector());
}

void HectorViewController::updateDistance()
{
  if ( position_properties_connected_ )
  {
    disconnect( distance_property_, SIGNAL( changed()), this, SLOT( onDistancePropertyChanged()));
  }
  distance_property_->setFloat((focus_point_property_->getVector() - eye_point_property_->getVector()).length());
  if ( position_properties_connected_ )
  {
    connect( distance_property_, SIGNAL( changed()), this, SLOT( onDistancePropertyChanged()));
  }
}

void HectorViewController::updateOrientation( float delta_yaw, float delta_pitch, float delta_roll )
{
  Ogre::Quaternion old_camera_orientation = camera_->getOrientation();

  Ogre::Quaternion yaw, pitch, roll;
  yaw.FromAngleAxis( Ogre::Radian( delta_yaw ), Ogre::Vector3::UNIT_Y );
  pitch.FromAngleAxis( Ogre::Radian( delta_pitch ), Ogre::Vector3::UNIT_X );
  roll.FromAngleAxis( Ogre::Radian( delta_roll ), Ogre::Vector3::UNIT_Z );
  Ogre::Quaternion orientation_change = yaw * pitch * roll;
  Ogre::Quaternion new_camera_orientation = old_camera_orientation * orientation_change;

  camera_->setOrientation( new_camera_orientation );
  Ogre::Vector3 new_eye_position =
    focus_point_property_->getVector() + distance_property_->getFloat() * new_camera_orientation.zAxis();
  eye_point_property_->setVector( new_eye_position );
  camera_->setPosition( new_eye_position );
}

void HectorViewController::setPropertiesFromCamera( const Ogre::Camera *camera )
{
  disconnectPositionProperties();
  eye_point_property_->setVector( camera->getPosition());
  focus_point_property_->setVector(
    camera->getPosition() - distance_property_->getFloat() * (camera->getOrientation() * Ogre::Vector3::UNIT_Z));
  connectPositionProperties();
}

void HectorViewController::handleMouseEvent2D( rviz::ViewportMouseEvent &evt )
{
  setStatus(
    "<b>Left-Click:</b> Rotate. <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b> Zoom. <b>Ctrl+D:</b> Switch to 3D. <b>Shift+Left-Click:</b> Move X/Y." );

  int32_t diff_x = 0;
  int32_t diff_y = 0;
  bool moved = false;

  if ( evt.type == QEvent::MouseButtonPress )
  {
    dragging_ = true;
    focal_shape_->getRootNode()->setVisible( true );
  }
  else if ( evt.type == QEvent::MouseButtonRelease )
  {
    focal_shape_->getRootNode()->setVisible( false );
    dragging_ = false;
  }
  else if ( dragging_ && evt.type == QEvent::MouseMove )
  {
    diff_x = evt.x - evt.last_x;
    diff_y = evt.y - evt.last_y;
    moved = true;
  }

  if ( evt.left() && !evt.shift())
  {
    setCursor( Rotate2D );
    angle_property_->add( static_cast<float>(diff_x) * 0.005f );
    camera_->setOrientation( Ogre::Quaternion( Ogre::Radian( angle_property_->getFloat()), Ogre::Vector3::UNIT_Z ));
  }
  else if ( evt.middle() || evt.left())
  {
    setCursor( MoveXY );
    float cosa = std::cos( angle_property_->getFloat());
    float sina = std::sin( angle_property_->getFloat());
    moveOnXYPlaneBy((-diff_x * cosa - diff_y * sina) * distance_property_->getFloat() / DISTANCE_SCALE_FACTOR,
                    (-diff_x * sina + diff_y * cosa) * distance_property_->getFloat() / DISTANCE_SCALE_FACTOR );
  }
  else if ( evt.right())
  {
    setCursor( Zoom );
    distance_property_->multiply( 1.0f + diff_y * 0.01f );
  }
  else
  {
    setCursor( evt.shift() ? MoveXY : Rotate2D );
  }

  if ( evt.wheel_delta != 0 )
  {
    distance_property_->multiply( 1.0f - static_cast<float>(evt.wheel_delta) * 0.001f );
    moved = true;
  }

  if ( moved ) context_->queueRender();
}

void HectorViewController::handleMouseEvent3D( rviz::ViewportMouseEvent &evt )
{
  setStatus(
    "<b>Left-Click:</b>  Rotate. <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b> Zoom.  <b>Shift</b>: Move faster. <b>Ctrl+D:</b> Switch to 2D." );

  float distance = distance_property_->getFloat();
  int diff_x = 0;
  int diff_y = 0;

  if ( evt.wheel_delta != 0 )
  {
    zoom( evt.wheel_delta * 0.001f * distance );
  }

  if ( evt.type == QEvent::MouseButtonPress )
  {
    dragging_ = true;
    focal_shape_->getRootNode()->setVisible( true );
  }
  else if ( evt.type == QEvent::MouseButtonRelease )
  {
    focal_shape_->getRootNode()->setVisible( false );
    dragging_ = false;
  }
  else if ( dragging_ && evt.type == QEvent::MouseMove )
  {
    diff_x = evt.x - evt.last_x;
    diff_y = evt.y - evt.last_y;
  }

  if ( evt.left() && !evt.shift())
  {
    setCursor( Rotate3D );
    updateOrientation( -diff_x * 0.005f, -diff_y * 0.005f, 0 );
  }
  else if ( evt.middle() || (evt.shift() && evt.left()))
  {
    setCursor( MoveXY );
    float fov_y = camera_->getFOVy().valueRadians();
    float fov_x = 2.f * atanf( tanf( fov_y / 2 ) * camera_->getAspectRatio());

    int width = camera_->getViewport()->getActualWidth();
    int height = camera_->getViewport()->getActualWidth();

    Ogre::Vector3 eye = eye_point_property_->getVector();
    Ogre::Vector3 translation;
    translation.x = diff_x == 0 ? 0 : -((float) diff_x / (float) width) * distance * tanf( fov_x / 2.0f ) * 2.0f;
    translation.y = diff_y == 0 ? 0 : ((float) diff_y / (float) height) * distance * tanf( fov_y / 2.0f ) * 2.0f;
    translation.z = 0;
    moveEyeWithFocusTo( eye + camera_->getOrientation() * translation, true, false );
  }
  else if ( evt.right())
  {
    if ( evt.shift())
    {
      setCursor( MoveZ );
      Ogre::Vector3 eye = eye_point_property_->getVector();
      Ogre::Vector3 translation = Ogre::Vector3( 0, 0, diff_y * 0.1f * distance / 10.0f );
      moveEyeWithFocusTo( eye + camera_->getOrientation() * translation, true, false );
    }
    else
    {
      setCursor( Zoom );
      zoom( -diff_y * 0.1f * distance / 10.0f );
    }
  }

  context_->queueRender();
}

namespace
{
bool getTransformOrLogError( rviz::FrameManager *frame_manager, const std::string &frame, const ros::Time &time,
                             Ogre::Vector3 &pos_out, Ogre::Quaternion &q_out )
{
  if ( !frame_manager->getTransform( frame, ros::Time(), pos_out,
                                     q_out ))
  {
    std::string error;
    if ( !frame_manager->transformHasProblems( frame, ros::Time(), error ))
    {
      error = "Unknown";
    }
    ROS_WARN_NAMED( "HectorViewController", "Could not get transform to locked frame! Reason: %s", error.c_str());
    return false;
  }
  return true;
}
}

void HectorViewController::update( float dt, float )
{
  updateCameraProperties( dt );

  if ( in_animation_ )
  {
    updateCamera();
    return;
  }
  if ( key_x_direction_ == 0 && key_y_direction_ == 0 )
  {
    updateCamera();
    return;
  }

  auto x = static_cast<float>(key_x_direction_);
  auto y = static_cast<float>(key_y_direction_);
  float length = std::abs( x ) + std::abs( y );
  float x_ratio = x / length;
  float y_ratio = y / length;

  x = x_ratio * max_movement_property_->getFloat() * dt;
  y = y_ratio * max_movement_property_->getFloat() * dt;

  if ( quick_mode_ )
  {
    x *= 2;
    y *= 2;
  }

  // This is y direction because we treat mouse up as y
  Ogre::Vector3 y_direction = eye_point_property_->getVector() - focus_point_property_->getVector();
  y_direction.z = 0;
  y_direction.normalise();
  Ogre::Vector3 x_direction = y_direction.crossProduct( up_vector_property_->getVector());
  x_direction.z = 0;
  x_direction.normalise();
  moveOnXYPlaneBy( -x * x_direction.x - y * y_direction.x, -x * x_direction.y - y * y_direction.y );
  updateCamera();
}

void HectorViewController::updateCameraProperties( float dt )
{
  bool camera_position_changed = false;
  Ogre::Vector3 focus = focus_point_property_->getVector(), eye = eye_point_property_->getVector();
  if ( in_animation_ )
  {
    ros::WallDuration elapsed = ros::WallTime::now() - animation_start_;
    float completed_percent = (float) elapsed.toSec() / animation_duration_property_->getFloat();
    if ( completed_percent < 1 )
    {
      // The completed percent is adjusted to make a smooth transition
      // using this sigmoid function: https://www.wolframalpha.com/input/?i=y%3D(1%2F(1%2Be%5E(-12(x-0.5)))),+from+0+to+1
      completed_percent = 1.f / (1 + expf( -12.f * (completed_percent - 0.5f)));
      Ogre::Vector3 animation_focus_goal_offset = animation_focus_goal_ - animation_focus_start_;
      Ogre::Vector3 animation_eye_goal_offset = animation_eye_goal_ - animation_eye_start_;
      focus = animation_focus_start_ + completed_percent * animation_focus_goal_offset;
      eye = animation_eye_start_ + completed_percent * animation_eye_goal_offset;
    }
    else if ( completed_percent >= 1 )
    {
      focus = animation_focus_goal_;
      eye = animation_eye_goal_;
      in_animation_ = false;
    }
    camera_position_changed = true;
  }

  if ( mode() == view_modes::Mode2D && !in_animation_ )
  {
    if ( in_mode_transition_ )
    {
      camera_->setProjectionType( Ogre::PT_ORTHOGRAPHIC );
      camera_->setFixedYawAxis( false );
      camera_->setOrientation( Ogre::Quaternion( Ogre::Radian( angle_property_->getFloat()), Ogre::Vector3::UNIT_Z ));
      in_mode_transition_ = false;
    }
    focus = focus_point_property_->getVector();
    camera_->setPosition( focus.x, focus.y, DISTANCE_SCALE_FACTOR );
    auto width = static_cast<float>(camera_->getViewport()->getActualWidth());
    auto height = static_cast<float>(camera_->getViewport()->getActualHeight());
    float scale = DISTANCE_SCALE_FACTOR / distance_property_->getFloat();

    Ogre::Matrix4 proj;
    rviz::buildScaledOrthoMatrix( proj, -width / scale / 2, width / scale / 2, -height / scale / 2, height / scale / 2,
                                  camera_->getNearClipDistance(), camera_->getFarClipDistance());
    camera_->setCustomProjectionMatrix( true, proj );
  }

  if ( is_frame_tracked_ )
  {
    Ogre::Vector3 current_position;
    Ogre::Quaternion current_orientation;
    if ( getTransformOrLogError( context_->getFrameManager(), tracked_frame_, ros::Time(),
                                 current_position, current_orientation ))
    {
      tracked_frame_remaining_position_difference_ += current_position - tracked_frame_last_position_;
      float weight = std::min<float>( tracked_frame_p_gain_property_->getFloat() * dt, 1 );
      Ogre::Vector3 position_diff = tracked_frame_remaining_position_difference_ * weight;
      tracked_frame_remaining_position_difference_ -= position_diff;

      tracked_frame_remaining_orientation_difference_ = tracked_frame_remaining_orientation_difference_ *
                                                        (tracked_frame_last_orientation_.UnitInverse() *
                                                         current_orientation);
      Ogre::Quaternion orientation_diff = Ogre::Quaternion::Slerp( weight, Ogre::Quaternion::IDENTITY,
                                                                   tracked_frame_remaining_orientation_difference_,
                                                                   true );
      tracked_frame_remaining_orientation_difference_ = Ogre::Quaternion::Slerp( 1 - weight,
                                                                                 Ogre::Quaternion::IDENTITY,
                                                                                 tracked_frame_remaining_orientation_difference_,
                                                                                 true );

      eye = orientation_diff * (eye - current_position) + current_position + position_diff;
      focus = orientation_diff * (focus - current_position) + current_position + position_diff;
      if ( in_animation_ )
      {
        // Correct animation goals
        animation_eye_goal_ =
          orientation_diff * (animation_eye_goal_ - current_position) + current_position + position_diff;
        animation_focus_goal_ =
          orientation_diff * (animation_focus_goal_ - current_position) + current_position + position_diff;
      }

      tracked_frame_last_position_ = current_position;
      tracked_frame_last_orientation_ = current_orientation;
      camera_position_changed = true;
    }
  }

  if ( camera_position_changed )
  {
    disconnectPositionProperties();
    eye_point_property_->setVector( eye );
    focus_point_property_->setVector( focus );
    updateDistance();
    connectPositionProperties();
  }
}
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( hector_rviz_plugins::HectorViewController, rviz::ViewController )
