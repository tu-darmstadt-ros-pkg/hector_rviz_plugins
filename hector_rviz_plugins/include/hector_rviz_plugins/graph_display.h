/*
 * Copyright (C) 2022  Stefan Fabian
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

#ifndef HECTOR_RVIZ_PLUGINS_GRAPH_DISPLAY_H
#define HECTOR_RVIZ_PLUGINS_GRAPH_DISPLAY_H

#include <hector_rviz_plugins_msgs/Graph.h>
#include <message_filters/subscriber.h>
#include <rviz/display.h>
#include <tf2_ros/message_filter.h>
#include <OgreMaterial.h>

namespace Ogre
{
class InstanceManager;
class SceneNode;
} // namespace Ogre

namespace rviz
{
class ColorProperty;
class IntProperty;
class RosTopicProperty;
} // namespace rviz

namespace hector_rviz_plugins
{

class GraphDisplay : public rviz::Display
{
  Q_OBJECT
public:
  GraphDisplay();

  ~GraphDisplay() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

public:
  void reset() override;

private slots:
  void updateTopic();

  void updateQueueSize();

  void updateUseNodeColors();

  void updateNodeColor();

private:
  void processGraph( const hector_rviz_plugins_msgs::Graph::ConstPtr &graph );

  void onFilteringFailed( const ros::MessageEvent<hector_rviz_plugins_msgs::Graph> &event,
                          tf2_ros::FilterFailureReason reason );

  void subscribe();

  void unsubscribe();

  std::unique_ptr<tf2_ros::MessageFilter<hector_rviz_plugins_msgs::Graph>> tf2_message_filter_;
  message_filters::Subscriber<hector_rviz_plugins_msgs::Graph> sub_;
  Ogre::MaterialPtr material_;
  Ogre::SceneNode *graph_node_;
  Ogre::InstanceManager *instance_manager_;
  std::vector<Ogre::SceneNode *> scene_nodes_;
  std::vector<Ogre::MaterialPtr> materials_;
  rviz::RosTopicProperty *topic_property_;
  rviz::IntProperty *queue_size_property_;
  rviz::BoolProperty *use_node_colors_property_;
  rviz::ColorProperty *node_color_property_;
};
} // namespace hector_rviz_plugins

#endif // HECTOR_RVIZ_PLUGINS_GRAPH_DISPLAY_H
