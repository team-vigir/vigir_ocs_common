/* 
 * Selection3DDisplayCustom class implementation.
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
#include "rviz/render_panel.h"

#include "selection_3d_display_custom.h"

namespace rviz
{

Selection3DDisplayCustom::Selection3DDisplayCustom()
  : Display()
  , has_new_transforms_( false )
  , time_since_last_transform_( 0.0f )
  , selection_marker_(NULL)
{
}

Selection3DDisplayCustom::~Selection3DDisplayCustom()
{
    this->scene_manager_->destroyQuery(mRayScnQuery);
}

void Selection3DDisplayCustom::onInitialize()
{
  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();
}

void Selection3DDisplayCustom::updateAlpha()
{
  context_->queueRender();
}

void Selection3DDisplayCustom::updateRobotDescription()
{
  if( isEnabled() )
  {
    load();
    context_->queueRender();
  }
}

void Selection3DDisplayCustom::updateVisualVisible()
{
  context_->queueRender();
}

void Selection3DDisplayCustom::updateCollisionVisible()
{
  context_->queueRender();
}

void Selection3DDisplayCustom::updateTfPrefix()
{
  clearStatuses();
  context_->queueRender();
}

void Selection3DDisplayCustom::load()
{

    std::cout << " 1" << std::endl;
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
    std::cout << " 2" << std::endl;

    // Create a box to be used as marker
    Ogre::Entity* lEntity = this->scene_manager_->createEntity("selection marker", Ogre::SceneManager::PT_CUBE);
    std::cout << " 3" << std::endl;
    selection_marker_ = this->scene_node_->createChildSceneNode();
    std::cout << " 4" << std::endl;
    selection_marker_->attachObject(lEntity);
    std::cout << " 5" << std::endl;
    // Change position and scale
    selection_marker_->setPosition(0.0f, 0.0f, 0.0f);
    selection_marker_->scale(0.001f,0.001f,0.001f);
    std::cout << " 6" << std::endl;

    //selection_marker_->setVisible( false );
}

void Selection3DDisplayCustom::onEnable()
{
    std::cout << "1" << std::endl;
    if(selection_marker_ == NULL)
    {
        load();
        std::cout << "2" << std::endl;

        //but we also want to set up our raySceneQuery after everything has been initialized
        mRayScnQuery = this->scene_manager_->createRayQuery(Ogre::Ray());
    }
    std::cout << "3" << std::endl;
    selection_marker_->setVisible( true );
}

void Selection3DDisplayCustom::onDisable()
{
    selection_marker_->setVisible( false );
}

void Selection3DDisplayCustom::update( float wall_dt, float ros_dt )
{
  time_since_last_transform_ += wall_dt;
  //float rate = update_rate_property_->getFloat();
  //bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

  context_->queueRender();
}

void Selection3DDisplayCustom::fixedFrameChanged()
{
  has_new_transforms_ = true;
}

void Selection3DDisplayCustom::clear()
{
}

void Selection3DDisplayCustom::reset()
{
  Display::reset();
  has_new_transforms_ = true;
}

void Selection3DDisplayCustom::createMarker(int xo, int yo, int x, int y)
{
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    //then send a raycast straight out from the camera at the mouse's position
    Ogre::Ray mouseRay = this->render_panel_->getCamera()->getCameraToViewportRay((float)x/win_width, (float)y/win_height);
    mRayScnQuery->setRay(mouseRay);

    //This next chunk finds the results of the raycast
    // 0 -> null
    // 1 -> ground
    // 2 and up -> node colision
    Ogre::RaySceneQueryResult& result = mRayScnQuery->execute();

    // we calculate the smallest (> 0) distance
    float min_distance = 1E+37;
    for( int i = 2; i < result.size(); i++ )
    {
        std::cout << "object picked [" << i << "] -> " << result[i].distance << std::endl;
        if( result[i].distance > 0 && result[i].distance < min_distance)
            min_distance = result[i].distance;
    }

    // but we only care about the first real colision for setting the marker position
    if( result.size() > 2)
    {
        Ogre::Vector3 position = mouseRay.getOrigin()+mouseRay.getDirection()*min_distance;
        std::cout << "object position: " << position.x << ", " << position.y << ", " << position.z << std::endl;
        selection_marker_->setPosition(position);
    }

    selection_marker_->setVisible( true );
}

void Selection3DDisplayCustom::setRenderPanel( rviz::RenderPanel* rp )
{
    render_panel_ = rp;
}

} // namespace rviz
