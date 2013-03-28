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
#include <OGRE/OgreMovableObject.h>

#include <urdf/model.h>

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
    //, has_new_transforms_( false )
    , time_since_last_transform_( 0.0f )
    , selection_marker_(NULL)
    //, mCurrentObject(NULL)
{
}

Selection3DDisplayCustom::~Selection3DDisplayCustom()
{
    delete raycast_utils_;
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
    /*Ogre::SceneNode* lightSceneNode = NULL;
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
    this->scene_manager_->setAmbientLight(ambientColour);*/

    // Create spheres to be used as markers
    Ogre::Entity* lEntity = this->scene_manager_->createEntity("selection marker", Ogre::SceneManager::PT_SPHERE);
    selection_marker_ = this->scene_node_->createChildSceneNode();
    selection_marker_->attachObject(lEntity);
    // Change position and scale
    selection_marker_->setPosition(0.0f, 0.0f, 0.0f);
    selection_marker_->scale(0.001f,0.001f,0.001f);

    selection_marker_->setVisible( false );

    lEntity = this->scene_manager_->createEntity("ROI selection marker initial", Ogre::SceneManager::PT_SPHERE);
    roi_marker_initial_ = this->scene_node_->createChildSceneNode();
    roi_marker_initial_->attachObject(lEntity);
    // Change position and scale
    roi_marker_initial_->setPosition(0.0f, 0.0f, 0.0f);
    roi_marker_initial_->scale(0.001f,0.001f,0.001f);

    roi_marker_initial_->setVisible( false );

    lEntity = this->scene_manager_->createEntity("ROI selection marker final", Ogre::SceneManager::PT_SPHERE);
    roi_marker_final_ = this->scene_node_->createChildSceneNode();
    roi_marker_final_->attachObject(lEntity);
    // Change position and scale
    roi_marker_final_->setPosition(0.0f, 0.0f, 0.0f);
    roi_marker_final_->scale(0.001f,0.001f,0.001f);

    roi_marker_final_->setVisible( false );

    lEntity = this->scene_manager_->createEntity("ROI selection marker box", Ogre::SceneManager::PT_CUBE);
    roi_marker_box_ = this->scene_node_->createChildSceneNode();
    roi_marker_box_->attachObject(lEntity);
    // Change position and scale
    roi_marker_box_->setPosition(0.0f, 0.0f, 0.0f);
    roi_marker_box_->scale(0.001f,0.001f,0.001f);

    roi_marker_box_->setVisible( false );
}

void Selection3DDisplayCustom::onEnable()
{
    if(selection_marker_ == NULL)
    {
        load();

        //but we also want to set up our raySceneQuery after everything has been initialized
        //mRayScnQuery = this->scene_manager_->createRayQuery(Ogre::Ray());
        // rayscenequery is initialized by raycastutils now
        raycast_utils_ = new RayCastUtils(this->scene_manager_);
    }
    //selection_marker_->setVisible( true );
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
    //has_new_transforms_ = true;
    context_->queueRender();
}

void Selection3DDisplayCustom::clear()
{
}

void Selection3DDisplayCustom::reset()
{
    Display::reset();
    //has_new_transforms_ = true;
}

void Selection3DDisplayCustom::createMarker(int xo, int yo, int x, int y)
{
    createMarker(x,y);
}

void Selection3DDisplayCustom::transform(Ogre::Vector3& position, Ogre::Quaternion& orientation)
{
    //std::cout << "POS bt: " << position.x << ", " << position.y << ", " << position.z << std::endl;
    // put all pose data into a tf stamped pose
    tf::Quaternion bt_orientation(0, 0, 0, 1);
    tf::Vector3 bt_position(0, 0, 0);

    std::string frame("/world");
    tf::Stamped<tf::Pose> pose_in(tf::Transform(bt_orientation,bt_position), ros::Time(), frame);
    tf::Stamped<tf::Pose> pose_out;

    // convert pose into new frame
    try
    {
      context_->getFrameManager()->getTFClient()->transformPose( fixed_frame_.toUtf8().constData(), pose_in, pose_out );
    }
    catch(tf::TransformException& e)
    {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s", frame.c_str(), fixed_frame_.toUtf8().constData(), e.what());
      return;
    }

    bt_position = pose_out.getOrigin();
    position = Ogre::Vector3(bt_position.x(), bt_position.y(), bt_position.z());
    std::cout << "POS transform: " << position.x << ", " << position.y << ", " << position.z << std::endl;

    bt_orientation = pose_out.getRotation();
    orientation = Ogre::Quaternion( bt_orientation.w(), bt_orientation.x(), bt_orientation.y(), bt_orientation.z() );
    std::cout << "QUAT transform: " << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << std::endl;
}

void Selection3DDisplayCustom::createMarker(int x, int y)
{
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    //then send a raycast straight out from the camera at the mouse's position
    Ogre::Ray mouseRay = this->render_panel_->getCamera()->getCameraToViewportRay((float)x/win_width, (float)y/win_height);

    Ogre::Vector3 position;

    Ogre::Vector3 pt;
    Ogre::Quaternion ot;
    transform(pt,ot);
    if(raycast_utils_->RayCastFromPoint(mouseRay,pt,ot,position))
    {
        selection_marker_->setPosition(position);
        selection_marker_->setVisible(true);
        Q_EMIT newSelection(position);
    }
    else
    {
        selection_marker_->setVisible(false);
    }


    /*
    // OLD CODE, ONLY LOOK FOR COLLISIONS WITH BOUNDING BOXES

    mRayScnQuery->setRay(mouseRay);

    //This next chunk finds the results of the raycast
    // 0 -> null
    // 1 -> ground
    // 2 and up -> node colision
    Ogre::RaySceneQueryResult& result = mRayScnQuery->execute();

    // we calculate the smallest (> 0) distance
    float min_distance = 1E+37;
    for( int i = 0; i < result.size(); i++ )
    {
        std::cout << "object picked [" << i << "] -> " << result[i].distance << std::endl;
        if( result[i].distance > 0 && result[i].distance < min_distance)
        {
            min_distance = result[i].distance;
            if(mCurrentObject)
                mCurrentObject->showBoundingBox(false);
            mCurrentObject = result[i].movable->getParentSceneNode();
        }
    }

    // but we only care about the first real colision for setting the marker position
    if( result.size() > 2)
    {
        Ogre::Vector3 position = mouseRay.getOrigin()+mouseRay.getDirection()*min_distance;
        std::cout << "object position: " << position.x << ", " << position.y << ", " << position.z << std::endl;
        selection_marker_->setPosition(position);
        mCurrentObject->showBoundingBox(true);
        selection_marker_->setVisible( true );
    }
    */
}

void Selection3DDisplayCustom::createROISelection(int xo, int yo, int x, int y)
{
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    bool failed = false;

    // calculate raycasting position
    Ogre::Ray mouseRayInitial = this->render_panel_->getCamera()->getCameraToViewportRay((float)xo/win_width, (float)yo/win_height);

    Ogre::Vector3 positionInitial;

    Ogre::Vector3 pt;
    Ogre::Quaternion ot;
    transform(pt,ot);
    if(raycast_utils_->RayCastFromPoint(mouseRayInitial,pt,ot,positionInitial))
    {
        Ogre::Ray mouseRayFinal = this->render_panel_->getCamera()->getCameraToViewportRay((float)x/win_width, (float)y/win_height);

        Ogre::Vector3 positionFinal;
        if(raycast_utils_->RayCastFromPoint(mouseRayFinal,pt,ot,positionFinal))
        {
            selection_marker_->setVisible(false);
            roi_marker_initial_->setPosition(positionInitial);
            roi_marker_initial_->setVisible(true);
            roi_marker_final_->setPosition(positionFinal);
            roi_marker_final_->setVisible(true);

            // emit signal here?
        }
        else
        {
            failed = true;
        }
    }
    else
    {
        failed = true;
    }

    if(failed)
    {
        selection_marker_->setVisible(true);
        roi_marker_initial_->setVisible(false);
        roi_marker_final_->setVisible(false);
    }
}

void Selection3DDisplayCustom::setRenderPanel( rviz::RenderPanel* rp )
{
    this->render_panel_ = rp;
}

} // namespace rviz
