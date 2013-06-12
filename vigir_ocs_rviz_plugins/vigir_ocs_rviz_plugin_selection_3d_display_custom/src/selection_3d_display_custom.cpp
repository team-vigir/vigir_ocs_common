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
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreFrustum.h>

#include <urdf/model.h>

#include "rviz/display_context.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/tf_link_updater.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"
#include "rviz/render_panel.h"
#include "rviz/validate_floats.h"
#include "rviz/view_manager.h"

#include "selection_3d_display_custom.h"

namespace rviz
{

Selection3DDisplayCustom::Selection3DDisplayCustom()
    : Display()
    //, has_new_transforms_( false )
    , time_since_last_transform_( 0.0f )
    , selection_marker_(NULL)
    //, mCurrentObject(NULL)
    , initialized_(false)
    , marker_scale_(0.0004f)
    , ray_initialized_(false)
    , raycast_request_mode_(-1)
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

    raycast_query_pub_ = nh_.advertise<flor_perception_msgs::RaycastRequest>( "/flor/worldmodel/ocs/dist_query_distance_request_world", 1, false );
    raycast_query_sub_ = nh_.subscribe<std_msgs::Float64>( "/flor/worldmodel/ocs/dist_query_distance_result", 5, &Selection3DDisplayCustom::processDistQuery, this );
    ocs_raycast_query_pub_ = nh_.advertise<flor_ocs_msgs::OCSRaycastRequest>( "/flor/ocs/dist_query_request", 1, false );
    ocs_raycast_query_sub_ = nh_.subscribe<flor_ocs_msgs::OCSRaycastRequest>( "/flor/ocs/dist_query_request", 5, &Selection3DDisplayCustom::processOCSDistQuery, this );
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
    Ogre::MaterialManager& lMaterialManager = Ogre::MaterialManager::getSingleton();
    Ogre::String lNameOfResourceGroup = "SelectionMaterials";

    Ogre::ResourceGroupManager& lRgMgr = Ogre::ResourceGroupManager::getSingleton();

    Ogre::String lMaterialName = lNameOfResourceGroup+"MarkerMaterial";

    if(!lRgMgr.resourceGroupExists(lNameOfResourceGroup))
    {
        lRgMgr.createResourceGroup(lNameOfResourceGroup);

        Ogre::MaterialPtr lMaterial = lMaterialManager.create(lMaterialName,lNameOfResourceGroup);
        Ogre::Technique* lFirstTechnique = lMaterial->getTechnique(0);
        Ogre::Pass* lFirstPass = lFirstTechnique->getPass(0);

        float transparency = 0.6f;
        Ogre::ColourValue lSelfIllumnationColour(0.0f, 0.0f, 0.0f, transparency);
        lFirstPass->setSelfIllumination(lSelfIllumnationColour);

        Ogre::ColourValue lDiffuseColour(1.0f, 1.0f, 0.0f, transparency);
        lFirstPass->setDiffuse(lDiffuseColour);

        Ogre::ColourValue lAmbientColour(0.4f, 0.4f, 0.1f, transparency);
        lFirstPass->setAmbient(lAmbientColour);

        Ogre::ColourValue lSpecularColour(1.0f, 1.0f, 1.0f, 1.0f);
        lFirstPass->setSpecular(lSpecularColour);

        Ogre::Real lShininess = 64.0f;
        lFirstPass->setShininess(lShininess);

        lFirstPass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        lFirstPass->setDepthWriteEnabled(false);
    }

    // Create spheres to be used as markers
    Ogre::Entity* lEntity = this->scene_manager_->createEntity("selection marker", Ogre::SceneManager::PT_SPHERE);
    //lEntity->setMaterialName(lMaterialName);
    selection_marker_ = this->scene_node_->createChildSceneNode();
    selection_marker_->attachObject(lEntity);
    // Change position and scale
    selection_marker_->setPosition(0.0f, 0.0f, 0.0f);
    selection_marker_->scale(0.001f,0.001f,0.001f);

    selection_marker_->setVisible( false );
    lEntity->setMaterialName(lMaterialName);

    lEntity = this->scene_manager_->createEntity("ROI selection marker final", Ogre::SceneManager::PT_SPHERE);
    //lEntity->setMaterialName(lMaterialName);
    roi_marker_final_ = this->scene_node_->createChildSceneNode();
    roi_marker_final_->attachObject(lEntity);
    // Change position and scale
    roi_marker_final_->setPosition(0.0f, 0.0f, 0.0f);
    roi_marker_final_->scale(0.001f,0.001f,0.001f);

    roi_marker_final_->setVisible( false );
    lEntity->setMaterialName(lMaterialName);

    lMaterialName = lNameOfResourceGroup+"MarkerBoxMaterial";

    if(!lRgMgr.resourceGroupExists(lNameOfResourceGroup))
    {
        lRgMgr.createResourceGroup(lNameOfResourceGroup);

        Ogre::MaterialPtr lMaterial = lMaterialManager.create(lMaterialName,lNameOfResourceGroup);
        Ogre::Technique* lFirstTechnique = lMaterial->getTechnique(0);
        Ogre::Pass* lFirstPass = lFirstTechnique->getPass(0);

        float transparency = 0.3f;
        Ogre::ColourValue lSelfIllumnationColour(0.1f, 0.0f, 0.0f, transparency);
        lFirstPass->setSelfIllumination(lSelfIllumnationColour);

        Ogre::ColourValue lDiffuseColour(1.0f, 0.4f, 0.4f, transparency);
        lFirstPass->setDiffuse(lDiffuseColour);

        Ogre::ColourValue lAmbientColour(0.4f, 0.1f, 0.1f, transparency);
        lFirstPass->setAmbient(lAmbientColour);

        Ogre::ColourValue lSpecularColour(1.0f, 1.0f, 1.0f, 1.0f);
        lFirstPass->setSpecular(lSpecularColour);

        Ogre::Real lShininess = 64.0f;
        lFirstPass->setShininess(lShininess);

        lFirstPass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        lFirstPass->setDepthWriteEnabled(false);
    }

    lEntity = this->scene_manager_->createEntity("ROI selection marker box", Ogre::SceneManager::PT_CUBE);
    lEntity->setMaterialName(lMaterialName);
    roi_marker_box_ = this->scene_node_->createChildSceneNode();
    roi_marker_box_->attachObject(lEntity);
    // Change position and scale
    roi_marker_box_->setPosition(100000.0f, 100000.0f, 100000.0f);
    roi_marker_box_->scale(0.001f,0.001f,0.001f);

    roi_marker_box_->setVisible( false );
    lEntity->setMaterialName(lMaterialName);

    // Create ground plane to be able to perform raycasting anywhere
    lEntity = this->scene_manager_->createEntity("ground plane", Ogre::SceneManager::PT_CUBE);
    //lEntity->setMaterialName(lMaterialName);
    ground_ = this->scene_node_->createChildSceneNode();
    ground_->attachObject(lEntity);
    // Change position and scale
    ground_->setPosition(0.0f, 0.0f, 0.0f);
    ground_->scale(10000.0f,10000.0f,0.00001f);
    ground_->setVisible( false );
}

void Selection3DDisplayCustom::onEnable()
{
    if(selection_marker_ == NULL)
    {
        load();

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

    if(initialized_)
    {
        //std::cout << "Update" << std::endl;
        // get transform from world to fixed frame
        Ogre::Quaternion selection_orientation(1,0,0,0);
        Ogre::Vector3 selection_position = selection_position_;
        transform(selection_position,selection_orientation,"/world",fixed_frame_.toUtf8().constData());
        //std::cout << "selection: " << selection_position.x << "," << selection_position.y << "," << selection_position.z << std::endl;

        //Ogre::Vector3 selection_position_roi = selection_position_roi_;
        //selection_position_roi = (ot * selection_position_roi) * pt;
        Ogre::Quaternion selection_orientation_roi(1,0,0,0);
        Ogre::Vector3 selection_position_roi = selection_position_roi_;
        transform(selection_position_roi,selection_orientation_roi,"/world",fixed_frame_.toUtf8().constData());
        //std::cout << selection_position_roi.x << "," << selection_position_roi.y << "," << selection_position_roi.z << std::endl;

        // get camera position to calculate selection marker
        //std::cout << "selection " << render_panel_->getCamera() << std::endl;
        Ogre::Vector3 camera_position = this->render_panel_->getCamera()->getPosition();
        //std::cout << this->context_->getSceneManager()->getCameras().size() << std::endl;
        //Ogre::SceneManager::CameraList camera_list = this->context_->getSceneManager()->getCameras();
        //Ogre::SceneManager::CameraIterator camera_list_iterator = this->context_->getSceneManager()->getCameraIterator();
        //while(camera_list_iterator.hasMoreElements())
        //{
        //    Ogre::Camera* temp_cam = camera_list_iterator.getNext();
        //    Ogre::Vector3 temp_pos = temp_cam->getDerivedPosition();
        //    std::cout << "it " << temp_pos.x << "," << temp_pos.y << "," << temp_pos.z << std::endl;
        //}

        //Ogre::Vector3 camera_position2 = this->context_->getSceneManager()->getCamera();
        //std::cout << "2 " << camera_position2.x << "," << camera_position2.y << "," << camera_position2.z << std::endl;

        if(this->render_panel_->getCamera()->getProjectionType() == Ogre::PT_ORTHOGRAPHIC) // if it's ortho, we need to calculate z again
        {
            Ogre::Matrix4 m = this->render_panel_->getCamera()->getProjectionMatrix();
            //float near   =  (1+m[2][3])/m[2][2];
            //float far    = -(1-m[2][3])/m[2][2];
            float bottom =  (1-m[1][3])/m[1][1];
            float top    = -(1+m[1][3])/m[1][1];
            //float left   = -(1+m[0][3])/m[0][0];
            //float right  =  (1-m[0][3])/m[0][0];
            //std::cout << "ortho:\n\t" << left << "\n\t" << right << "\n\t" << bottom << "\n\t" << top << "\n\t" << near << "\n\t" << far << std::endl;
            camera_position.z = fabs(bottom)+fabs(top);
        }

        //std::cout << "camera: " << camera_position.x << "," << camera_position.y << "," << camera_position.z << std::endl;

        // get the current fov
        float size = marker_scale_;

        // calculate distance from markers
        if(validateFloats(selection_position))
        {
            float distance_selection = camera_position.distance(selection_position);
            //std::cout << distance_selection << std::endl;
            if(distance_selection != std::numeric_limits<float>::infinity() && !(distance_selection != distance_selection)) // check for inf and nan
            {
                if(!selection_position.isNaN()) selection_marker_->setPosition(selection_position);
                float scale_selection = 2.0f * distance_selection * tan(this->render_panel_->getCamera()->getFOVy().valueRadians() / 2.0f);//distance_selection*tan(radians)*0.002f;
                //std::cout << scale_selection << std::endl;
                if(scale_selection > 0.0f) selection_marker_->setScale(scale_selection*size,scale_selection*size,scale_selection*size);

                if(validateFloats(selection_position_roi))
                {
                    float distance_selection_roi = camera_position.distance(selection_position_roi);
                    //std::cout << distance_selection_roi << std::endl; // <<< inf
                    if(distance_selection_roi != std::numeric_limits<float>::infinity() && !(distance_selection_roi != distance_selection_roi))
                    {
                        if(!selection_position_roi.isNaN()) roi_marker_final_->setPosition(selection_position_roi);
                        float scale_selection_roi = 2.0f * distance_selection_roi * tan(this->render_panel_->getCamera()->getFOVy().valueRadians() / 2.0f);// = distance_selection_roi*tan(radians)*0.002f;
                        //std::cout << scale_selection_roi << std::endl;
                        if(scale_selection_roi > 0.0f) roi_marker_final_->setScale(scale_selection_roi*size,scale_selection_roi*size,scale_selection_roi*size);

                        Ogre::Vector3 box_position = (selection_position+selection_position_roi)/2.0f;

                        if(!box_position.isNaN()) roi_marker_box_->setPosition(box_position);
                        if(!selection_orientation.isNaN()) roi_marker_box_->setOrientation(selection_orientation);
                    }
                }
            }
        }
    }

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
    createMarker(false,x,y);
}

void Selection3DDisplayCustom::transform(Ogre::Vector3& position, Ogre::Quaternion& orientation, const char* from_frame, const char* to_frame)
{
    //std::cout << "POS bt: " << position.x << ", " << position.y << ", " << position.z << std::endl;
    // put all pose data into a tf stamped pose
    tf::Quaternion bt_orientation(orientation.x, orientation.y, orientation.z, orientation.w);
    tf::Vector3 bt_position(position.x, position.y, position.z);

    std::string frame(from_frame);
    tf::Stamped<tf::Pose> pose_in(tf::Transform(bt_orientation,bt_position), ros::Time(), frame);
    tf::Stamped<tf::Pose> pose_out;

    // convert pose into new frame
    try
    {
      context_->getFrameManager()->getTFClient()->transformPose( to_frame, pose_in, pose_out );
    }
    catch(tf::TransformException& e)
    {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s", from_frame, to_frame, e.what());
      return;
    }

    bt_position = pose_out.getOrigin();
    position = Ogre::Vector3(bt_position.x(), bt_position.y(), bt_position.z());
    //std::cout << "POS transform: " << position.x << ", " << position.y << ", " << position.z << std::endl;

    bt_orientation = pose_out.getRotation();
    orientation = Ogre::Quaternion( bt_orientation.w(), bt_orientation.x(), bt_orientation.y(), bt_orientation.z() );
    //std::cout << "QUAT transform: " << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << std::endl;
}

void Selection3DDisplayCustom::createMarker(bool, int x, int y)
{
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    //then send a raycast straight out from the camera at the mouse's position
    Ogre::Ray mouseRay = this->render_panel_->getCamera()->getCameraToViewportRay((float)x/win_width, (float)y/win_height);

    Ogre::Vector3 position;

    Ogre::Vector3 pt(0,0,0);
    Ogre::Quaternion ot(1,0,0,0);
    transform(pt,ot,"/world",fixed_frame_.toUtf8().constData());
    int type;
    if(raycast_utils_->RayCastFromPoint(mouseRay,pt,ot,position,type))
    {
        initialized_ = true;
        selection_marker_->setPosition(position);
        selection_marker_->setVisible(true);
        //Q_EMIT newSelection(position);

        std::cout << "POS transform in: " << position.x << ", " << position.y << ", " << position.z << std::endl;

        Ogre::Quaternion orientation(1,0,0,0);
        transform(position,orientation,fixed_frame_.toUtf8().constData(),"/world");
        //selection_position_ = (ot * position) * pt;
        selection_position_ = position;
        Q_EMIT newSelection(selection_position_);

        std::cout << "POS transform out: " << position.x << ", " << position.y << ", " << position  .z << std::endl;
    }
    else
    {
        initialized_ = false;
        selection_marker_->setVisible(false);
    }
    roi_marker_final_->setVisible(false);
    roi_marker_box_->setVisible(false);
    roi_marker_box_->setScale(0.0000001f,0.0000001f,0.0000001f);
}

void Selection3DDisplayCustom::createROISelection(bool, int x, int y)
{
    if(!initialized_)
        return;

    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    bool failed = false;

    //then send a raycast straight out from the camera at the mouse's position
    Ogre::Ray mouseRayFinal = this->render_panel_->getCamera()->getCameraToViewportRay((float)x/win_width, (float)y/win_height);

    Ogre::Vector3 positionFinal;

    Ogre::Vector3 pt(0,0,0);
    Ogre::Quaternion ot(1,0,0,0);
    transform(pt,ot,"/world",fixed_frame_.toUtf8().constData());
    int type;
    if(raycast_utils_->RayCastFromPoint(mouseRayFinal,pt,ot,positionFinal,type))
    {
        selection_marker_->setVisible(true);
        roi_marker_final_->setPosition(positionFinal);
        roi_marker_final_->setVisible(true);

        Ogre::Vector3 initial_position = selection_marker_->getPosition();
        Ogre::Vector3 box_position = (initial_position+positionFinal)/2.0f;
        roi_marker_box_->setPosition(box_position);
        Ogre::Vector3 box_scale;
        box_scale.x = fabs(positionFinal.x-initial_position.x)/100.0f;
        box_scale.y = fabs(positionFinal.y-initial_position.y)/100.0f;
        box_scale.z = fabs(positionFinal.z-initial_position.z)/100.0f;
        roi_marker_box_->setScale(box_scale);
        std::cout << "box_position: " << box_position.x << ", " << box_position.y << ", " << box_position.z << std::endl;
        std::cout << "box_scale: " << box_scale.x << ", " << box_scale.y << ", " << box_scale.z << std::endl;
        roi_marker_box_->setVisible(true);

        Ogre::Quaternion orientation(1,0,0,0);
        transform(positionFinal,orientation,fixed_frame_.toUtf8().constData(),"/world");
        selection_position_roi_ = positionFinal;

        // emit signal here?
    }
    else
    {
        selection_marker_->setVisible(false);
        roi_marker_final_->setVisible(false);
        roi_marker_box_->setVisible(false);
        roi_marker_box_->setScale(0.0000001f,0.0000001f,0.0000001f);
    }
}

void Selection3DDisplayCustom::setRenderPanel( rviz::RenderPanel* rp )
{
    this->render_panel_ = rp;
}

void Selection3DDisplayCustom::resetSelection()
{

    selection_marker_->setVisible(false);
    selection_marker_->setScale(0.0000001f,0.0000001f,0.0000001f);
    roi_marker_final_->setVisible(false);
    roi_marker_final_->setScale(0.0000001f,0.0000001f,0.0000001f);
    roi_marker_box_->setVisible(false);
    roi_marker_box_->setScale(0.0000001f,0.0000001f,0.0000001f);
}

void Selection3DDisplayCustom::setMarkerScale(float scale)
{
    marker_scale_ = scale;
}

void Selection3DDisplayCustom::queryPosition( int x, int y, Ogre::Vector3& world_position )
{
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    //then send a raycast straight out from the camera at the mouse's position
    Ogre::Ray mouseRay = this->render_panel_->getCamera()->getCameraToViewportRay((float)x/win_width, (float)y/win_height);

    Ogre::Vector3 position;

    Ogre::Vector3 pt(0,0,0);
    Ogre::Quaternion ot(1,0,0,0);
    transform(pt,ot,"/world",fixed_frame_.toUtf8().constData());
    int type = -1;
    if(raycast_utils_->RayCastFromPoint(mouseRay,pt,ot,position,type))
    {
        Ogre::Quaternion orientation(1,0,0,0);
        transform(position,orientation,fixed_frame_.toUtf8().constData(),"/world");
        world_position = position;
    }
}

void Selection3DDisplayCustom::queryContext( int x, int y )
{
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    //then send a raycast straight out from the camera at the mouse's position
    Ogre::Ray mouseRay = this->render_panel_->getCamera()->getCameraToViewportRay((float)x/win_width, (float)y/win_height);

    Ogre::Vector3 position;

    Ogre::Vector3 pt(0,0,0);
    Ogre::Quaternion ot(1,0,0,0);
    transform(pt,ot,"/world",fixed_frame_.toUtf8().constData());
    int type = -1;
    if(raycast_utils_->RayCastFromPoint(mouseRay,pt,ot,position,type))
    {
        // type 0 -> UNKNOWN ENTITY
        // type 1 -> POINT CLOUD
        // type 2 -> WAYPOINT
        // type 3 -> TEMPLATE
        Q_EMIT setContext(type);
    }
    else
    {
        Q_EMIT setContext(-1); // NO_HIT
    }
}

void Selection3DDisplayCustom::setMarkerPosition(float x, float y, float z)
{
    Ogre::Vector3 position(x,y,z);

    initialized_ = true;
    selection_marker_->setPosition(position);
    selection_marker_->setVisible(true);
    //Q_EMIT newSelection(position);

    std::cout << "POS transform in: " << position.x << ", " << position.y << ", " << position.z << std::endl;

    //selection_position_ = (ot * position) * pt;
    selection_position_ = position;
    Q_EMIT newSelection(selection_position_);

    roi_marker_final_->setVisible(false);
    roi_marker_box_->setVisible(false);
    roi_marker_box_->setScale(0.0000001f,0.0000001f,0.0000001f);
}

Ogre::Vector3 Selection3DDisplayCustom::calculateRaycastPosition(double distance)
{
    Ogre::Vector3 origin = last_ray_.getOrigin();
    Ogre::Vector3 direction = last_ray_.getDirection();
    return origin+(direction*distance);
}

void Selection3DDisplayCustom::processOCSDistQuery( const flor_ocs_msgs::OCSRaycastRequest::ConstPtr &request )
{
    Ogre::Vector3 origin(request->origin.x,request->origin.y,request->origin.z);
    Ogre::Vector3 direction(request->direction.x,request->direction.y,request->direction.z);
    last_ray_.setOrigin(origin);
    last_ray_.setDirection(direction);
    raycast_request_mode_ = request->mode;
    Q_EMIT setSelectionRay(last_ray_);
}

void Selection3DDisplayCustom::processDistQuery( const std_msgs::Float64::ConstPtr& distance )
{
    if(distance->data < 0)
        return;

    // prevent losing data in case OCS takes longer than onboard to respond
    //while(raycast_request_mode_ == -1)
    //    sleep(0.05);

    if(raycast_request_mode_ == RAYCAST_SELECTION)
    {
        /*if(distance->data < 0)
        {
            initialized_ = false;
            selection_marker_->setVisible(false);
            return;
        }*/

        Ogre::Vector3 positionFinal = calculateRaycastPosition(distance->data);

        setMarkerPosition(positionFinal.x,positionFinal.y,positionFinal.z);

        raycast_request_mode_ = -1;
    }
    else if(raycast_request_mode_ == RAYCAST_SELECTION_ROI)
    {
        if(!initialized_)
            return;

        /*if(distance->data < 0)
        {
            //selection_marker_->setVisible(false);
            roi_marker_final_->setVisible(false);
            roi_marker_box_->setVisible(false);
            roi_marker_box_->setScale(0.0000001f,0.0000001f,0.0000001f);
            return;
        }*/

        Ogre::Vector3 positionFinal = calculateRaycastPosition(distance->data);

        selection_marker_->setVisible(true);
        roi_marker_final_->setPosition(positionFinal);
        roi_marker_final_->setVisible(true);

        Ogre::Vector3 initial_position = selection_marker_->getPosition();
        Ogre::Vector3 box_position = (initial_position+positionFinal)/2.0f;
        roi_marker_box_->setPosition(box_position);
        Ogre::Vector3 box_scale;
        box_scale.x = fabs(positionFinal.x-initial_position.x)/100.0f;
        box_scale.y = fabs(positionFinal.y-initial_position.y)/100.0f;
        box_scale.z = fabs(positionFinal.z-initial_position.z)/100.0f;
        roi_marker_box_->setScale(box_scale);
        std::cout << "box_position: " << box_position.x << ", " << box_position.y << ", " << box_position.z << std::endl;
        std::cout << "box_scale: " << box_scale.x << ", " << box_scale.y << ", " << box_scale.z << std::endl;
        roi_marker_box_->setVisible(true);

        Ogre::Quaternion orientation(1,0,0,0);
        transform(positionFinal,orientation,fixed_frame_.toUtf8().constData(),"/world");
        selection_position_roi_ = positionFinal;

        raycast_request_mode_ = -1;
    }
}

void Selection3DDisplayCustom::publishRayRequest(Ogre::Vector3 origin, Ogre::Vector3 direction)
{
    flor_perception_msgs::RaycastRequest request;

    request.origin.x = origin.x;
    request.origin.y = origin.y;
    request.origin.z = origin.z;

    request.direction.x = direction.x;
    request.direction.y = direction.y;
    request.direction.z = direction.z;

    raycast_query_pub_.publish(request);
}

void Selection3DDisplayCustom::publishOCSRayRequest(int mode, Ogre::Vector3 origin, Ogre::Vector3 direction)
{
    flor_ocs_msgs::OCSRaycastRequest request;

    request.origin.x = origin.x;
    request.origin.y = origin.y;
    request.origin.z = origin.z;

    request.direction.x = direction.x;
    request.direction.y = direction.y;
    request.direction.z = direction.z;

    request.mode = mode;

    ocs_raycast_query_pub_.publish(request);
}

void Selection3DDisplayCustom::raycastRequest(bool, int x, int y)
{
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    //then send a raycast straight out from the camera at the mouse's position
    Ogre::Ray mouseRay = this->render_panel_->getCamera()->getCameraToViewportRay((float)x/win_width, (float)y/win_height);

    // send ray data to other instances of OCS
    publishOCSRayRequest(RAYCAST_SELECTION, mouseRay.getOrigin(), mouseRay.getDirection());

    // send ray data to onboard
    publishRayRequest(mouseRay.getOrigin(), mouseRay.getDirection());
}

void Selection3DDisplayCustom::raycastRequestROI(bool, int x, int y)
{
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    //then send a raycast straight out from the camera at the mouse's position
    Ogre::Ray mouseRay = this->render_panel_->getCamera()->getCameraToViewportRay((float)x/win_width, (float)y/win_height);

    // send ray data to other instances of OCS
    publishOCSRayRequest(RAYCAST_SELECTION_ROI, mouseRay.getOrigin(), mouseRay.getDirection());

    // send ray data to onboard
    publishRayRequest(mouseRay.getOrigin(), mouseRay.getDirection());

}

} // namespace rviz
