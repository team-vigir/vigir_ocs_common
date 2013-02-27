/* 
 * TemplateDisplayCustom class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on the rviz image display class.
 *
 * Latest changes (12/11/2012):
 * - fixed segfault issues
 */
/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <urdf/model.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/tf_link_updater.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"

#include "template_display_custom.h"

namespace rviz
{

void linkUpdaterStatusFunction( StatusProperty::Level level,
                                const std::string& link_name,
                                const std::string& text,
                                TemplateDisplayCustom* display )
{
  display->setStatus( level, QString::fromStdString( link_name ), QString::fromStdString( text ));
}

TemplateDisplayCustom::TemplateDisplayCustom()
  : Display()
  , has_new_transforms_( false )
  , time_since_last_transform_( 0.0f )
{
  visual_enabled_property_ = new Property( "Visual Enabled", true,
                                           "Whether to display the visual representation of the robot.",
                                           this, SLOT( updateVisualVisible() ));

  collision_enabled_property_ = new Property( "Collision Enabled", false,
                                              "Whether to display the collision representation of the robot.",
                                              this, SLOT( updateCollisionVisible() ));

  update_rate_property_ = new FloatProperty( "Update Interval", 0,
                                             "Interval at which to update the links, in seconds. "
                                             " 0 means to update every update cycle.",
                                             this );
  update_rate_property_->setMin( 0 );

  alpha_property_ = new FloatProperty( "Alpha", 1,
                                       "Amount of transparency to apply to the links.",
                                       this, SLOT( updateAlpha() ));
  alpha_property_->setMin( 0.0 );
  alpha_property_->setMax( 1.0 );

  robot_description_property_ = new StringProperty( "Robot Description", "robot_description",
                                                    "Name of the parameter to search for to load the robot description.",
                                                    this, SLOT( updateRobotDescription() ));

  tf_prefix_property_ = new StringProperty( "TF Prefix", "",
                                            "Robot Model normally assumes the link name is the same as the tf frame name. "
                                            " This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
                                            this, SLOT( updateTfPrefix() ));
}

TemplateDisplayCustom::~TemplateDisplayCustom()
{
  //delete robot_;
}

void TemplateDisplayCustom::onInitialize()
{
  //robot_ = new Robot( scene_node_, context_, "Robot: " + getName().toStdString(), this );

  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();
}

void TemplateDisplayCustom::updateAlpha()
{
  //robot_->setAlpha( alpha_property_->getFloat() );
  context_->queueRender();
}

void TemplateDisplayCustom::updateRobotDescription()
{
  if( isEnabled() )
  {
    load();
    context_->queueRender();
  }
}

void TemplateDisplayCustom::updateVisualVisible()
{
  //robot_->setVisualVisible( visual_enabled_property_->getValue().toBool() );
  context_->queueRender();
}

void TemplateDisplayCustom::updateCollisionVisible()
{
  //robot_->setCollisionVisible( collision_enabled_property_->getValue().toBool() );
  context_->queueRender();
}

void TemplateDisplayCustom::updateTfPrefix()
{
  clearStatuses();
  context_->queueRender();
}

void TemplateDisplayCustom::load()
{
	Ogre::SceneNode* lightSceneNode = NULL;
	Ogre::Light* light = this->scene_manager_->createLight();

	// I can set some attributes of the light.
	// The basic light type can be : 
	//		pointlight (like a candle?)
	//		spotlight (kind of 'conic' light)
	//		directional light (like the sun in an outdoor scene).
	// Directional light is like parallel rays coming from 1 direction.
	light->setType(Ogre::Light::LT_DIRECTIONAL);

	// Here I choose the color of the light.
	// The diffuse color is the main color of the light.
	// The specular color is its color when reflected on an imperfect surface.
	// For example, when my bald head skin reflect the sun, it makes a bright round of specular color.
	//
	// The final color of an object also depends on its material.
	// Color values vary between 0.0(minimum) to 1.0 (maximum).
	light->setDiffuseColour(0.8f, 0.8f, 0.8f); // this will be a red light
	light->setSpecularColour(1.0f, 1.0f, 1.0f);// color of 'reflected' light

	lightSceneNode = this->scene_node_->createChildSceneNode();
	lightSceneNode->attachObject(light);
 
	// I add an ambient color. The ambient color is managed in the scenemanager.
	// If you want to learn more about ambient/specular/diffuse color, check the 'basic material tutorial'
	// from this serie.
	Ogre::ColourValue ambientColour(0.2f, 0.2f, 0.2f, 1.0f);
	this->scene_manager_->setAmbientLight(ambientColour);

	// create a new resource group for all our templates
	Ogre::String nameOfResourceGroup = "templates";
	Ogre::ResourceGroupManager& resourceManager = Ogre::ResourceGroupManager::getSingleton();
	resourceManager.createResourceGroup(nameOfResourceGroup);

	// set the template directory path as root of our resource manager
	// need a global config.xml file to define this sort of stuff that is going to be used by multiple plugins and packages
	Ogre::String templatePath = "/opt/vigir/vigir_control/vigir_grasping/templates/";
	resourceManager.addResourceLocation(templatePath, "FileSystem", nameOfResourceGroup, false);

	// parse scripts
	resourceManager.initialiseResourceGroup(nameOfResourceGroup);

	// loads files into our resource manager
	resourceManager.loadResourceGroup(nameOfResourceGroup);

	// Now the loaded Mesh is available from its ResourceGroup,
	// as well as from the Ogre::MeshManager. A shared pointer to
	// it can be accessed by : Ogre::MeshManager::getSingleton().getByName(name_of_the_mesh);

	// create entity for mesh and attach it to the scene node
	Ogre::String lNameOfTheMesh = "vehicle/steering_wheel_mm.mesh";
	Ogre::Entity* lEntity = this->scene_manager_->createEntity(lNameOfTheMesh);
	Ogre::SceneNode* lNode = this->scene_node_->createChildSceneNode();
	lNode->attachObject(lEntity);
	// change position and scale (from mm to m)
	lNode->translate(1.0f, 0.0f, 0.0f);
	lNode->scale(0.001f,0.001f,0.001f);
	// The loaded mesh will be white. This is normal.
}

void TemplateDisplayCustom::onEnable()
{
  load();
  //robot_->setVisible( true );
}

void TemplateDisplayCustom::onDisable()
{
  //robot_->setVisible( false );
  clear();
}

void TemplateDisplayCustom::update( float wall_dt, float ros_dt )
{
  time_since_last_transform_ += wall_dt;
  float rate = update_rate_property_->getFloat();
  bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

  context_->queueRender();
}

void TemplateDisplayCustom::fixedFrameChanged()
{
  has_new_transforms_ = true;
}

void TemplateDisplayCustom::clear()
{
  //robot_->clear();
  clearStatuses();
  robot_description_.clear();
}

void TemplateDisplayCustom::reset()
{
  Display::reset();
  has_new_transforms_ = true;
}

} // namespace rviz
