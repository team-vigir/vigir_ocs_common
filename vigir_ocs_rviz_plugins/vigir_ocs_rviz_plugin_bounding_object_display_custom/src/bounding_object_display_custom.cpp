/*
 * BoundingObjectDisplayCustom class implementation.
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
#include "rviz/properties/color_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/render_panel.h"
#include "rviz/validate_floats.h"
#include "rviz/view_manager.h"
#include "rviz/visualization_manager.h"
#include "robot_display_custom.h"

#include "bounding_object_display_custom.h"

namespace rviz
{

BoundingObjectDisplayCustom::BoundingObjectDisplayCustom()
    : Display()
    , time_since_last_transform_( 0.0f )
    , bounding_object_(NULL)
    , initialized_(false)
{
    name_property_ = new StringProperty( "Name", "",
                                         "Name of the object.", this , SLOT(updateName()));

    color_property_ = new ColorProperty( "Color", QColor( 25, 255, 0 ),
                                         "Color to draw the path.", this, SLOT( updateObjectProperties() ) );

    alpha_property_ = new FloatProperty( "Alpha", 0.6f,
                                         "Amount of transparency to apply to the path.", this, SLOT( updateObjectProperties() ) );

    Ogre::Vector3 default_scale(0.003f,0.003f,0.003f);
    scale_property_ = new VectorProperty( "Scale", default_scale,
                                         "Size of the object.", this, SLOT( updateObjectProperties() ) );

    topic_property_ = new RosTopicProperty( "Pose Topic", "",
                                            QString::fromStdString( ros::message_traits::datatype<geometry_msgs::PoseStamped>() ),
                                            "geometry_msgs::PoseStamped topic to subscribe to.",
                                            this, SLOT( updateTopic() ));

    offset_property_ = new VectorProperty( "Position Offset", Ogre::Vector3::ZERO,
                                             "Offset of the bounding object in relation to the object",
                                             this, SLOT( updateObjectProperties() ) );    
    offset_property_->setReadOnly( true );

    position_property_ = new VectorProperty( "Position", Ogre::Vector3::ZERO,
                                             "position of the bounding object in relation to the object",
                                             this, SLOT( updateObjectProperties() ) );
    rotation_property_ = new QuaternionProperty("Rotation", Ogre::Quaternion::IDENTITY,"rotation of bounding object",this,SLOT(updateObjectProperties()));

}

BoundingObjectDisplayCustom::~BoundingObjectDisplayCustom()
{
}

void BoundingObjectDisplayCustom::updateName()
{
    if(bounding_object_ == NULL)
    {        
        load();
        initialized_ = true;       
    }
}

void BoundingObjectDisplayCustom::onInitialize()
{
    context_->getSceneManager()->addRenderQueueListener(this);
}

void BoundingObjectDisplayCustom::setPose()
{
    Ogre::Quaternion orientation(rotation_property_->getQuaternion().w,
                                 rotation_property_->getQuaternion().x,
                                 rotation_property_->getQuaternion().y,
                                 rotation_property_->getQuaternion().z);
    Ogre::Vector3 rotated_offset = orientation * offset_property_->getVector();
    bounding_object_->setPosition(position_property_->getVector().x + rotated_offset.x,
                                  position_property_->getVector().y + rotated_offset.y,
                                  position_property_->getVector().z+ rotated_offset.z);
    bounding_object_->setOrientation(orientation);
}

void BoundingObjectDisplayCustom::updatePose( const geometry_msgs::PoseStamped::ConstPtr& pose )
{
    pose_ = pose->pose;


    // set properties
    position_property_->setVector(Ogre::Vector3(pose_.position.x,pose_.position.y,pose_.position.z));
    rotation_property_->setQuaternion(Ogre::Quaternion(pose_.orientation.w,pose_.orientation.x,pose_.orientation.y,pose_.orientation.z));
    setPose();
}

void BoundingObjectDisplayCustom::updateObjectProperties()
{
    // update transformations
    setPose();

    bounding_object_->setScale(scale_property_->getVector().x,scale_property_->getVector().y,scale_property_->getVector().z);

    // update color/alpha
    Ogre::Technique* lFirstTechnique = bounding_object_material_->getTechnique(0);
    Ogre::Pass* lFirstPass = lFirstTechnique->getPass(0);

    Ogre::ColourValue lSelfIllumnationColour(0.0f, 0.0f, 0.0f, alpha_property_->getFloat());
    lFirstPass->setSelfIllumination(lSelfIllumnationColour);

    Ogre::ColourValue lDiffuseColour(color_property_->getColor().redF(), color_property_->getColor().greenF(), color_property_->getColor().blueF(), alpha_property_->getFloat());
    lFirstPass->setDiffuse(lDiffuseColour);

    Ogre::ColourValue lAmbientColour(color_property_->getColor().redF()/2.0f, color_property_->getColor().greenF()/2.0f, color_property_->getColor().blueF()/2.0f, alpha_property_->getFloat());
    lFirstPass->setAmbient(lAmbientColour);

    Ogre::ColourValue lSpecularColour(1.0f, 1.0f, 1.0f, 1.0f);
    lFirstPass->setSpecular(lSpecularColour);

    Ogre::Real lShininess = 64.0f;
    lFirstPass->setShininess(lShininess);

    lFirstPass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    lFirstPass->setDepthWriteEnabled(false);

    context_->queueRender();
}

void BoundingObjectDisplayCustom::updateTopic()
{
    unsubscribe();
    subscribe();
}

void BoundingObjectDisplayCustom::subscribe()
{
    if ( !isEnabled() )
    {
        return;
    }

    if( !topic_property_->getTopic().isEmpty() )
    {
        try
        {
            pose_sub_ = nh_.subscribe( topic_property_->getTopicStd(), 1, &BoundingObjectDisplayCustom::updatePose, this );
            setStatus( StatusProperty::Ok, "Topic", "OK" );
        }
        catch( ros::Exception& e )
        {
            setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
        }
    }
}

void BoundingObjectDisplayCustom::unsubscribe()
{
    pose_sub_.shutdown();
}

void BoundingObjectDisplayCustom::load()
{    
    static int count = 0;
    std::stringstream ss;
    ss << "BoundingObject" << count++ << "Group";
    Ogre::MaterialManager& lMaterialManager = Ogre::MaterialManager::getSingleton();
    Ogre::String lNameOfResourceGroup =  ss.str();

    Ogre::ResourceGroupManager& lRgMgr = Ogre::ResourceGroupManager::getSingleton();

    Ogre::String lMaterialName = lNameOfResourceGroup+"BoundingObjectMaterial";

    if(!lRgMgr.resourceGroupExists(lNameOfResourceGroup))
    {
        lRgMgr.createResourceGroup(lNameOfResourceGroup);

        bounding_object_material_ = lMaterialManager.create(lMaterialName,lNameOfResourceGroup);
        Ogre::Technique* lFirstTechnique = bounding_object_material_->getTechnique(0);
        Ogre::Pass* lFirstPass = lFirstTechnique->getPass(0);

        Ogre::ColourValue lSelfIllumnationColour(0.0f, 0.0f, 0.0f, alpha_property_->getFloat());
        lFirstPass->setSelfIllumination(lSelfIllumnationColour);

        Ogre::ColourValue lDiffuseColour(color_property_->getColor().redF(), color_property_->getColor().greenF(), color_property_->getColor().blueF(), alpha_property_->getFloat());
        lFirstPass->setDiffuse(lDiffuseColour);

        Ogre::ColourValue lAmbientColour(color_property_->getColor().redF()/2.0f, color_property_->getColor().greenF()/2.0f, color_property_->getColor().blueF()/2.0f, alpha_property_->getFloat());
        lFirstPass->setAmbient(lAmbientColour);

        Ogre::ColourValue lSpecularColour(1.0f, 1.0f, 1.0f, 1.0f);
        lFirstPass->setSpecular(lSpecularColour);

        Ogre::Real lShininess = 64.0f;
        lFirstPass->setShininess(lShininess);

        lFirstPass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        lFirstPass->setDepthWriteEnabled(false);
    }

    Ogre::Entity* lEntity = this->scene_manager_->createEntity(ss.str()+name_property_->getStdString(), Ogre::SceneManager::PT_SPHERE);
    lEntity->setUserAny(Ogre::Any(ss.str()+name_property_->getStdString()));
    bounding_object_ = this->scene_node_->createChildSceneNode();
    bounding_object_->attachObject(lEntity);

    // Change position and scale
    bounding_object_->setPosition(0.0f, 0.0f, 0.0f);
    bounding_object_->scale(scale_property_->getVector().x,scale_property_->getVector().y,scale_property_->getVector().z);

    bounding_object_->setVisible( true );
    lEntity->setMaterialName(lMaterialName);

    // can't change name after it's been initialized
    name_property_->setReadOnly(true);
}

void BoundingObjectDisplayCustom::onEnable()
{
    subscribe();
}

void BoundingObjectDisplayCustom::onDisable()
{
    unsubscribe();
}

void BoundingObjectDisplayCustom::preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt )
{
    if(initialized_)
    {
        /*int view_id = 0;//((rviz::VisualizationManager*)context_)->getActiveViewID();

        for(int i = 0; i < render_panel_list_.size(); i++)
            if(render_panel_list_[i]->getRenderWindow() == (Ogre::RenderWindow*)evt.source)
                view_id = i;

        //std::cout << "Update " << view_id << std::endl;

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
        Ogre::Vector3 camera_position = this->render_panel_list_[view_id]->getCamera()->getPosition();
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
       // std::cout << "Camera: " << camera_position.x << "," << camera_position.y << "," << camera_position.z << std::endl;

        if(this->render_panel_list_[view_id]->getCamera()->getProjectionType() == Ogre::PT_ORTHOGRAPHIC) // if it's ortho, we need to calculate z again
        {
            Ogre::Matrix4 m = this->render_panel_list_[view_id]->getCamera()->getProjectionMatrix();
            float near   =  (1+m[2][3])/m[2][2];
            float far    = -(1-m[2][3])/m[2][2];
            float bottom =  (1-m[1][3])/m[1][1];
            float top    = -(1+m[1][3])/m[1][1];
            float left   = -(1+m[0][3])/m[0][0];
            float right  =  (1-m[0][3])/m[0][0];
            //std::cout << "ortho:\n\t" << left << "\n\t" << right << "\n\t" << bottom << "\n\t" << top << "\n\t" << near << "\n\t" << far << std::endl;
            if(this->render_panel_list_[view_id]->getCamera()->getPosition().z == 500)
                camera_position.z = fabs(bottom)+fabs(top);
            else if(this->render_panel_list_[view_id]->getCamera()->getPosition().y == -500)
                camera_position.y = fabs(bottom)+fabs(top);
            else if(this->render_panel_list_[view_id]->getCamera()->getPosition().x == 500)
                camera_position.x = fabs(bottom)+fabs(top);
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
                if(!selection_position.isNaN()) bounding_object_->setPosition(selection_position);
                float scale_selection = 2.0f * distance_selection * tan(this->render_panel_list_[view_id]->getCamera()->getFOVy().valueRadians() / 2.0f);//distance_selection*tan(radians)*0.002f;
                //std::cout << scale_selection << std::endl;
                if(scale_selection > 0.0f) bounding_object_->setScale(scale_selection*size,scale_selection*size,scale_selection*size);

                if(validateFloats(selection_position_roi))
                {
                    float distance_selection_roi = camera_position.distance(selection_position_roi);
                    //std::cout << distance_selection_roi << std::endl; // <<< inf
                    if(distance_selection_roi != std::numeric_limits<float>::infinity() && !(distance_selection_roi != distance_selection_roi))
                    {
                        if(!selection_position_roi.isNaN()) roi_marker_final_->setPosition(selection_position_roi);
                        float scale_selection_roi = 2.0f * distance_selection_roi * tan(this->render_panel_list_[view_id]->getCamera()->getFOVy().valueRadians() / 2.0f);// = distance_selection_roi*tan(radians)*0.002f;
                        //std::cout << scale_selection_roi << std::endl;
                        if(scale_selection_roi > 0.0f) roi_marker_final_->setScale(scale_selection_roi*size,scale_selection_roi*size,scale_selection_roi*size);

                        Ogre::Vector3 box_position = (selection_position+selection_position_roi)/2.0f;

                        if(!box_position.isNaN()) roi_marker_box_->setPosition(box_position);
                        if(!selection_orientation.isNaN()) roi_marker_box_->setOrientation(selection_orientation);
                    }
                }
            }
        }*/
    }

    context_->queueRender();
}

void BoundingObjectDisplayCustom::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{

}

void BoundingObjectDisplayCustom::renderQueueStarted(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation)
{

}

void BoundingObjectDisplayCustom::renderQueueEnded(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& repeatThisInvocation)
{

}

void BoundingObjectDisplayCustom::update( float wall_dt, float ros_dt )
{
    time_since_last_transform_ += wall_dt;
}

void BoundingObjectDisplayCustom::fixedFrameChanged()
{
    //has_new_transforms_ = true;
    context_->queueRender();
}

void BoundingObjectDisplayCustom::clear()
{
}

void BoundingObjectDisplayCustom::reset()
{
    Display::reset();
    //has_new_transforms_ = true;
}

void BoundingObjectDisplayCustom::transform(Ogre::Vector3& position, Ogre::Quaternion& orientation, const char* from_frame, const char* to_frame)
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

void BoundingObjectDisplayCustom::setRenderPanel( rviz::RenderPanel* rp )
{
    if(std::find(render_panel_list_.begin(),render_panel_list_.end(),rp) != render_panel_list_.end())
    {
        this->render_panel_ = rp;
        this->render_panel_->getRenderWindow()->addListener( this );
    }
    else
    {
        this->render_panel_list_.push_back(rp);
        this->render_panel_ = rp;
        this->render_panel_->getRenderWindow()->addListener( this );
    }
}

} // namespace rviz
