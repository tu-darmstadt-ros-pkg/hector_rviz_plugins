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

#include "hector_rviz_plugins/graph_display.h"

#include <rviz/display_context.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>

#include <OgreEntity.h>
#include <OgreInstancedEntity.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace hector_rviz_plugins
{
GraphDisplay::GraphDisplay()
{
  topic_property_ = new rviz::RosTopicProperty(
      "Topic", "graph",
      QString::fromStdString( ros::message_traits::datatype<hector_rviz_plugins_msgs::Graph>() ),
      "hector_rviz_plugins_msgs::Graph topic to subscribe to.", this, SLOT( updateTopic() ) );

  queue_size_property_ = new rviz::IntProperty(
      "Queue Size", 1,
      "The queue size for the subscriber. May be useful if tf data is delayed significantly from "
      "the graph data at the cost of increased memory footprint.",
      this, SLOT( updateQueueSize() ) );
  queue_size_property_->setMin( 0 );

  use_node_colors_property_ = new rviz::BoolProperty(
      "Use Node Colors", true,
      "Turn off to display all nodes in the same color for improved performance.", this,
      SLOT( updateUseNodeColors() ) );

  node_color_property_ = new rviz::ColorProperty(
      "Color", Qt::darkYellow, "The color to use for all nodes.", this, SLOT( updateNodeColor() ) );
  node_color_property_->setHidden( true );
}

GraphDisplay::~GraphDisplay() { }

void GraphDisplay::onInitialize()
{
  tf2_message_filter_ = std::make_unique<tf2_ros::MessageFilter<hector_rviz_plugins_msgs::Graph>>(
      *context_->getTF2BufferPtr(), fixed_frame_.toStdString(), queue_size_property_->getInt(),
      update_nh_ );
  tf2_message_filter_->registerCallback( &GraphDisplay::processGraph, this );
  tf2_message_filter_->registerFailureCallback(
      [this]( const auto &ev, auto reason ) { onFilteringFailed( ev, reason ); } );
  tf2_message_filter_->connectInput( sub_ );

  //  render_object_ = scene_manager_->createManualObject();
  //  scene_manager_->getRootSceneNode()->createChildSceneNode( "graph" )->attachObject( render_object_ );
  graph_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode( "GraphNode" );
  material_ = Ogre::MaterialManager::getSingleton().create(
      "GraphDisplay_NodeMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  material_->setDiffuse( node_color_property_->getOgreColor() );
}

void GraphDisplay::onEnable()
{
  subscribe();
  graph_node_->setVisible( true, false );
}

void GraphDisplay::onDisable()
{
  unsubscribe();
  graph_node_->setVisible( false );
}

void GraphDisplay::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
}

void GraphDisplay::updateQueueSize()
{
  tf2_message_filter_->setQueueSize( queue_size_property_->getInt() );
  subscribe();
}
void GraphDisplay::updateUseNodeColors()
{
  reset();
  node_color_property_->setHidden( use_node_colors_property_->getBool() );
}

void GraphDisplay::updateNodeColor()
{
  material_->setDiffuse( node_color_property_->getOgreColor() );
}

void GraphDisplay::subscribe()
{
  if ( !isEnabled() )
    return;

  std::string topic = topic_property_->getTopicStd();
  if ( topic.empty() )
    return;

  try {
    sub_.subscribe( update_nh_, topic, queue_size_property_->getInt() );
    setStatus( rviz::StatusProperty::Ok, "Topic", "OK" );
  } catch ( ros::Exception &e ) {
    setStatus( rviz::StatusProperty::Error, "Topic", QString( "Failed to subscribe: " ) + e.what() );
  }
}

void GraphDisplay::unsubscribe() { sub_.unsubscribe(); }

void GraphDisplay::reset()
{
  graph_node_->removeAndDestroyAllChildren();
  scene_nodes_.clear();
  materials_.clear();
}

namespace
{
bool isEmptyColor( const std_msgs::ColorRGBA &color )
{
  return color.r == 0 && color.g == 0 && color.b == 0 && color.a == 0;
}

void updatePositionIfNecessary( Ogre::SceneNode *node, Ogre::Real x, Ogre::Real y, Ogre::Real z )
{
  const Ogre::Vector3 &pos = node->getPosition();
  if ( std::abs( pos.x - x ) < 1e-3 && std::abs( pos.y - y ) < 1e-3 && std::abs( pos.z - z ) < 1e-3 )
    return;
  node->setPosition( x, y, z );
}

} // namespace

void GraphDisplay::processGraph(
    const hector_rviz_plugins_msgs::Graph_<std::allocator<void>>::ConstPtr &graph )
{
  bool use_node_colors = use_node_colors_property_->getBool();
  auto &material_manager = Ogre::MaterialManager::getSingleton();
  int i = 0;
  for ( const auto &node : graph->nodes ) {
    if ( i >= scene_nodes_.size() ) {
      Ogre::SceneNode *scene_node = graph_node_->createChildSceneNode();
      Ogre::Entity *entity = scene_manager_->createEntity( Ogre::SceneManager::PT_SPHERE );
      if ( use_node_colors ) {
        Ogre::MaterialPtr material =
            material_manager.create( "GraphDisplay_NodeMaterial" + std::to_string( i ),
                                     Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
        material->setDiffuse( node.color.r, node.color.g, node.color.b, node.color.a );
        entity->setMaterial( material );
        materials_.push_back( material );
      } else {
        entity->setMaterial( material_ );
      }
      scene_node->attachObject( entity );
      scene_node->setScale( 0.001, 0.001, 0.001 );
      scene_nodes_.push_back( scene_node );
    }
    Ogre::SceneNode *scene_node = scene_nodes_[i];
    updatePositionIfNecessary( scene_node, node.position.x, node.position.y, node.position.z );
    if ( use_node_colors ) {
      if ( isEmptyColor( node.color ) )
        materials_[i]->setDiffuse( 1.0, 1.0, 1.0, 0.5 );
      else
        materials_[i]->setDiffuse( node.color.r, node.color.g, node.color.b, node.color.a );
    }
    scene_node->setVisible( true );
    ++i;
  }
  for ( ; i < scene_nodes_.size(); ++i ) { scene_nodes_[i]->setVisible( false ); }
}
} // namespace hector_rviz_plugins

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( hector_rviz_plugins::GraphDisplay, rviz::Display )
