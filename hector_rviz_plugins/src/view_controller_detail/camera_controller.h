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

#ifndef HECTOR_RVIZ_PLUGINS_CAMERA_CONTROLLER_H
#define HECTOR_RVIZ_PLUGINS_CAMERA_CONTROLLER_H

#include <rviz/display_context.h>

namespace hector_rviz_plugins
{

class CameraAnimator
{
public:
  explicit CameraAnimator( rviz::DisplayContext *context ) : context_( context ) { }

  void animate( const Ogre::Vector3 &eye_start, const Ogre::Vector3 &eye_goal,
                const Ogre::Vector3 &focus_start, const Ogre::Vector3 &focus_goal )
  {
    animation_eye_start_ = eye_start;
    animation_eye_goal_ = eye_goal;

    animation_focus_start_ = focus_start;
    animation_focus_goal_ = focus_goal;

    // Transform animation positions into tracked frame if we track a frame
    if ( is_frame_tracked_ )
    {
      if ( !transformToTrackedFrame( animation_eye_start_, animation_eye_goal_,
                                     animation_focus_start_, animation_focus_goal_ ))
        return;
      clearTrackingOffsets();
    }

    animation_start_ = ros::WallTime::now();

    in_animation_ = true;
  }

  static void transform( const Ogre::Quaternion &rotation, const Ogre::Vector3 &translation, Ogre::Vector3 &vec )
  {
    vec = rotation * vec + translation;
  }

  template<typename... Args>
  static void transform( const Ogre::Quaternion &rotation, const Ogre::Vector3 &translation, Ogre::Vector3 &vec,
                         Args&&... args )
  {
    vec = rotation * vec + translation;
    // Template hack to allow passing multiple vectors to transform
    transform( rotation, translation, std::forward<Args>(args)... );
  }

  template<typename... Args>
  bool transformToTrackedFrame( Args&&... args )
  {
    Ogre::Vector3 translation;
    Ogre::Quaternion rotation;
    if ( !getTransformOrLogError( context_->getFrameManager(), tracked_frame_, ros::Time(), translation, rotation ))
    {
      in_animation_ = false;
      return false;
    }
    rotation = rotation.Inverse();
    translation = -(rotation * translation);
    transform( rotation, translation, std::forward<Args>(args)...);
    return true;
  }

  template<typename... Args>
  bool transformToFixedFrame( Args&&... args )
  {
    Ogre::Vector3 translation;
    Ogre::Quaternion rotation;
    if ( !getTransformOrLogError( context_->getFrameManager(), tracked_frame_, ros::Time(), translation, rotation ))
    {
      in_animation_ = false;
      return false;
    }
    transform( rotation, translation, std::forward<Args>(args)... );
    return true;
  }

  void clearTrackingOffsets()
  {
    tracked_frame_remaining_position_difference_ = Ogre::Vector3::ZERO;
    tracked_frame_remaining_orientation_difference_ = Ogre::Quaternion::IDENTITY;
    Ogre::Vector3 current_position;
    Ogre::Quaternion current_orientation;
    if ( !getTransformOrLogError( context_->getFrameManager(), tracked_frame_, ros::Time(),
                                  current_position, current_orientation ))
      return;
    tracked_frame_last_position_ = current_position;
    tracked_frame_last_orientation_ = current_orientation;
  }

  void trackFrame( const std::string &name )
  {
    // Transform animation in fixed frame
    if ( in_animation_ )
    {
      transformToFixedFrame( animation_eye_start_, animation_eye_goal_,
                             animation_focus_start_, animation_focus_goal_ );
    }
    is_frame_tracked_ = !name.empty();
    tracked_frame_ = name;
    if ( is_frame_tracked_ )
    {
      context_->getFrameManager()->getTransform( name, ros::Time(), tracked_frame_last_position_,
                                                 tracked_frame_last_orientation_ );
      clearTrackingOffsets();
      // Transform animation back to new tracked frame
      if ( in_animation_ )
      {
        transformToTrackedFrame( animation_eye_start_, animation_eye_goal_,
                                 animation_focus_start_, animation_focus_goal_ );
      }
    }
  }

  void stopTracking()
  {
    is_frame_tracked_ = false;
  }

  bool isFrameTracked() const { return is_frame_tracked_; }

  std::string trackedFrame() const { return tracked_frame_; }

  float pGain() const { return p_gain_; }

  void setPGain( float value ) { p_gain_ = value; }

  static bool getTransformOrLogError( rviz::FrameManager *frame_manager,
                                      const std::string &frame, const ros::Time &time,
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
      ROS_WARN_NAMED( "HectorViewController", "Could not get transform to tracked frame! Reason: %s", error.c_str());
      return false;
    }
    return true;
  }

  bool updateCamera( Ogre::Vector3 &eye, Ogre::Vector3 &focus, float dt )
  {
    if ( !in_animation_ && !is_frame_tracked_ ) return false;

    bool in_animation = in_animation_;
    if ( in_animation_ )
    {
      ros::WallDuration elapsed = ros::WallTime::now() - animation_start_;
      float completed_percent = (float) elapsed.toSec() / animation_duration_;
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
    }

    if ( !is_frame_tracked_ ) return true;

    Ogre::Vector3 current_position;
    Ogre::Quaternion current_orientation;
    if ( !getTransformOrLogError( context_->getFrameManager(), tracked_frame_, ros::Time(),
                                  current_position, current_orientation ))
      return true;

    if ( in_animation )
    {
      // Transform animation vector to fixed frame
      transform( current_orientation, current_position, focus, eye );
    }
    Ogre::Quaternion orientation_diff = (tracked_frame_last_orientation_.UnitInverse() *
                                         current_orientation);
    tracked_frame_remaining_position_difference_ = orientation_diff * tracked_frame_remaining_position_difference_;
    tracked_frame_remaining_position_difference_ += current_position - tracked_frame_last_position_;
    float weight = std::min<float>( p_gain_ * dt, 1 );
    Ogre::Vector3 position_delta = tracked_frame_remaining_position_difference_ * weight;
    tracked_frame_remaining_position_difference_ -= position_delta;

    // Only use rotation around z-axis
    orientation_diff.x = 0;
    orientation_diff.y = 0;
    orientation_diff.normalise();
    tracked_frame_remaining_orientation_difference_ =
      orientation_diff * tracked_frame_remaining_orientation_difference_;
    tracked_frame_remaining_orientation_difference_.x = 0;
    tracked_frame_remaining_orientation_difference_.y = 0;
    tracked_frame_remaining_orientation_difference_.normalise();
    Ogre::Quaternion orientation_delta = Ogre::Quaternion::Slerp( weight, Ogre::Quaternion::IDENTITY,
                                                                  tracked_frame_remaining_orientation_difference_,
                                                                  true );
    tracked_frame_remaining_orientation_difference_ = Ogre::Quaternion::Slerp( 1 - weight,
                                                                               Ogre::Quaternion::IDENTITY,
                                                                               tracked_frame_remaining_orientation_difference_,
                                                                               true );

    Ogre::Vector3 offset = (tracked_frame_last_position_ + current_position) / 2;
    eye = orientation_delta * (eye - tracked_frame_last_position_) + tracked_frame_last_position_ + position_delta;
    focus = orientation_delta * (focus - tracked_frame_last_position_) + tracked_frame_last_position_ + position_delta;

    tracked_frame_last_position_ = current_position;
    tracked_frame_last_orientation_ = current_orientation;

    return true;
  }

  void stop()
  {
    in_animation_ = false;
  }

  void setAnimationDuration( float value ) { animation_duration_ = value; }

private:
  Ogre::Vector3 animation_eye_start_;
  Ogre::Vector3 animation_eye_goal_;
  Ogre::Vector3 animation_focus_start_;
  Ogre::Vector3 animation_focus_goal_;
  ros::WallTime animation_start_;
  float animation_duration_ = 0.4;
  bool in_animation_ = false;

  std::string tracked_frame_;
  Ogre::Vector3 tracked_frame_last_position_;
  Ogre::Vector3 tracked_frame_remaining_position_difference_;
  Ogre::Quaternion tracked_frame_last_orientation_;
  Ogre::Quaternion tracked_frame_remaining_orientation_difference_;
  float p_gain_ = 3;
  bool is_frame_tracked_ = false;

  rviz::DisplayContext *context_;
};
}

#endif //HECTOR_RVIZ_PLUGINS_CAMERA_CONTROLLER_H
