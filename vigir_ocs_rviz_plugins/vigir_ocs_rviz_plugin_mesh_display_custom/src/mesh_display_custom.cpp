/*
 * MeshDisplayCustom class implementation.
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

#include "mesh_display_custom.h"

namespace rviz
{

MeshDisplayCustom::MeshDisplayCustom()
    : Display()
    , time_since_last_transform_( 0.0f )
    , mesh_node_(NULL)
    , projector_node_(NULL)
    , manual_object_(NULL)
    , initialized_(false)
{
    color_property_ = new ColorProperty( "Color", QColor( 25, 255, 0 ),
                                         "Color to draw the path.", this, SLOT( updateObjectProperties() ) );

    alpha_property_ = new FloatProperty( "Alpha", 0.6f,
                                         "Amount of transparency.", this, SLOT( updateObjectProperties() ) );

    mesh_topic_property_ = new RosTopicProperty( "Mesh Topic", "",
                                            QString::fromStdString( ros::message_traits::datatype<shape_msgs::Mesh>() ),
                                            "shape_msgs::Mesh topic to subscribe to.",
                                            this, SLOT( updateTopic() ));

    // this shouldn't necessarily be here, and we should get this from a camera topic with camera info
    position_property_ = new VectorProperty( "Projector Position", Ogre::Vector3::ZERO,
                                             "position of the texture projector object in /world",
                                             this, SLOT( updateObjectProperties() ) );
    rotation_property_ = new QuaternionProperty("Projector Rotation", Ogre::Quaternion::IDENTITY,"rotation of the texture projector object",this,SLOT(updateObjectProperties()));

}

MeshDisplayCustom::~MeshDisplayCustom()
{
}

void MeshDisplayCustom::onInitialize()
{
    context_->getSceneManager()->addRenderQueueListener(this);
}

void MeshDisplayCustom::createProjector()
{
    decal_frustum_ = new Ogre::Frustum();

    projector_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    projector_node_->attachObject(decal_frustum_);

    filter_frustum_ = new Ogre::Frustum();
    filter_frustum_->setProjectionType(Ogre::PT_ORTHOGRAPHIC);

    Ogre::SceneNode* filter_node = projector_node_->createChildSceneNode();
    filter_node->attachObject(filter_frustum_);
    filter_node->setOrientation(Ogre::Quaternion(Ogre::Degree(90),Ogre::Vector3::UNIT_Y));
}

void MeshDisplayCustom::addDecalToMaterial(const Ogre::String& matName)
{
    Ogre::MaterialPtr mat = (Ogre::MaterialPtr)Ogre::MaterialManager::getSingleton().getByName(matName);
    Ogre::Pass* pass = mat->getTechnique(0)->createPass();

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthBias(1);
    pass->setLightingEnabled(false);

    // TEMPORARY TEST, SHOULD RECEIVE IMAGE TOPIC AND USE THAT AS TEXTURE
    Ogre::String resource_group_name = "picturesfolder";
    Ogre::ResourceGroupManager& resource_manager = Ogre::ResourceGroupManager::getSingleton();
    if(!resource_manager.resourceGroupExists(resource_group_name))
    {
        resource_manager.createResourceGroup(resource_group_name);
        resource_manager.addResourceLocation("/home/vigir/Pictures/", "FileSystem", resource_group_name, false);
        resource_manager.initialiseResourceGroup(resource_group_name);
    }
    // loads files into our resource manager
    resource_manager.loadResourceGroup(resource_group_name);

    Ogre::TextureUnitState* tex_state = pass->createTextureUnitState("Decal.png");
    tex_state->setProjectiveTexturing(true, decal_frustum_);
    tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    tex_state->setTextureFiltering(Ogre::FO_POINT, Ogre::FO_LINEAR, Ogre::FO_NONE);

    tex_state = pass->createTextureUnitState("Decal_filter.png");
    tex_state->setProjectiveTexturing(true, filter_frustum_);
    tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    tex_state->setTextureFiltering(Ogre::TFO_NONE);
}

void MeshDisplayCustom::setPose()
{
    if(projector_node_ == NULL)
        return;

    projector_node_->setPosition(position_property_->getVector().x,
                            position_property_->getVector().y,
                            position_property_->getVector().z);
    projector_node_->setOrientation(rotation_property_->getQuaternion().w,
                               rotation_property_->getQuaternion().x,
                               rotation_property_->getQuaternion().y,
                               rotation_property_->getQuaternion().z);
}

void MeshDisplayCustom::updateMesh( const shape_msgs::Mesh::ConstPtr& mesh )
{
    // create our scenenode and material
    load();

    // set properties
    setPose();

    if (!manual_object_)
    {
        static uint32_t count = 0;
        std::stringstream ss;
        ss << "MeshObject" << count++;
        manual_object_ = context_->getSceneManager()->createManualObject(ss.str());
        mesh_node_->attachObject(manual_object_);
    }

    // If we have the same number of tris as previously, just update the object
    if (last_mesh_.vertices.size() > 0 && mesh->vertices.size() == last_mesh_.vertices.size())
    {
        manual_object_->beginUpdate(0);
    }
    else // Otherwise clear it and begin anew
    {
        manual_object_->clear();
        manual_object_->estimateVertexCount(mesh->vertices.size());
        manual_object_->begin(mesh_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    }

    const std::vector<geometry_msgs::Point>& points = mesh->vertices;
    for(size_t i = 0; i < mesh->triangles.size(); i++)
    {

        std::vector<Ogre::Vector3> corners(3);
        for(size_t c = 0; c < 3; c++)
        {
            corners[c] = Ogre::Vector3(points[mesh->triangles[i].vertex_indices[c]].x, points[mesh->triangles[i].vertex_indices[c]].y, points[mesh->triangles[i].vertex_indices[c]].z);
        }
        Ogre::Vector3 normal = (corners[1] - corners[0]).crossProduct(corners[2] - corners[0]);
        normal.normalise();

        for(size_t c = 0; c < 3; c++)
        {
            manual_object_->position(corners[c]);
            manual_object_->normal(normal);
        }
    }

    manual_object_->end();

    last_mesh_ = *mesh;
}

void MeshDisplayCustom::updateObjectProperties()
{
    // update transformations
    setPose();

    // update color/alpha
    Ogre::Technique* technique = mesh_material_->getTechnique(0);
    Ogre::Pass* pass = technique->getPass(0);

    Ogre::ColourValue self_illumination_color(0.0f, 0.0f, 0.0f, alpha_property_->getFloat());
    pass->setSelfIllumination(self_illumination_color);

    Ogre::ColourValue diffuse_color(color_property_->getColor().redF(), color_property_->getColor().greenF(), color_property_->getColor().blueF(), alpha_property_->getFloat());
    pass->setDiffuse(diffuse_color);

    Ogre::ColourValue ambient_color(color_property_->getColor().redF()/2.0f, color_property_->getColor().greenF()/2.0f, color_property_->getColor().blueF()/2.0f, alpha_property_->getFloat());
    pass->setAmbient(ambient_color);

    Ogre::ColourValue specular_color(1.0f, 1.0f, 1.0f, 1.0f);
    pass->setSpecular(specular_color);

    Ogre::Real shininess = 64.0f;
    pass->setShininess(shininess);

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);

    context_->queueRender();
}

void MeshDisplayCustom::updateTopic()
{
    unsubscribe();
    subscribe();
}

void MeshDisplayCustom::subscribe()
{
    if ( !isEnabled() )
    {
        return;
    }

    if( !mesh_topic_property_->getTopic().isEmpty() )
    {
        try
        {
            pose_sub_ = nh_.subscribe( mesh_topic_property_->getTopicStd(), 1, &MeshDisplayCustom::updateMesh, this );
            setStatus( StatusProperty::Ok, "Topic", "OK" );
        }
        catch( ros::Exception& e )
        {
            setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
        }
    }
}

void MeshDisplayCustom::unsubscribe()
{
    pose_sub_.shutdown();
}

void MeshDisplayCustom::load()
{
    if(mesh_node_ != NULL)
        return;

    static int count = 0;
    std::stringstream ss;
    ss << "MeshNode" << count++ << "Group";
    Ogre::MaterialManager& material_manager = Ogre::MaterialManager::getSingleton();
    Ogre::String resource_group_name =  ss.str();

    Ogre::ResourceGroupManager& rg_mgr = Ogre::ResourceGroupManager::getSingleton();

    Ogre::String material_name = resource_group_name+"MeshMaterial";

    if(!rg_mgr.resourceGroupExists(resource_group_name))
    {
        rg_mgr.createResourceGroup(resource_group_name);

        mesh_material_ = material_manager.create(material_name,resource_group_name);
        Ogre::Technique* technique = mesh_material_->getTechnique(0);
        Ogre::Pass* pass = technique->getPass(0);

        Ogre::ColourValue self_illumnation_color(0.0f, 0.0f, 0.0f, alpha_property_->getFloat());
        pass->setSelfIllumination(self_illumnation_color);

        Ogre::ColourValue diffuse_color(color_property_->getColor().redF(), color_property_->getColor().greenF(), color_property_->getColor().blueF(), alpha_property_->getFloat());
        pass->setDiffuse(diffuse_color);

        Ogre::ColourValue ambient_color(color_property_->getColor().redF()/2.0f, color_property_->getColor().greenF()/2.0f, color_property_->getColor().blueF()/2.0f, alpha_property_->getFloat());
        pass->setAmbient(ambient_color);

        Ogre::ColourValue specular_color(1.0f, 1.0f, 1.0f, 1.0f);
        pass->setSpecular(specular_color);

        Ogre::Real shininess = 64.0f;
        pass->setShininess(shininess);

        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        pass->setDepthWriteEnabled(false);

        mesh_material_->setCullingMode(Ogre::CULL_NONE);
    }

    mesh_node_ = this->scene_node_->createChildSceneNode();

    createProjector();

    addDecalToMaterial(material_name);
}

void MeshDisplayCustom::onEnable()
{
    subscribe();
}

void MeshDisplayCustom::onDisable()
{
    unsubscribe();
}

void MeshDisplayCustom::preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt )
{

}

void MeshDisplayCustom::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{

}

void MeshDisplayCustom::renderQueueStarted(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation)
{

}

void MeshDisplayCustom::renderQueueEnded(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& repeatThisInvocation)
{

}

void MeshDisplayCustom::update( float wall_dt, float ros_dt )
{
    time_since_last_transform_ += wall_dt;

    // just added automatic rotation to make it easier  to test things
    if(projector_node_ != NULL)
    {
        projector_node_->rotate(Ogre::Vector3::UNIT_Y, Ogre::Degree(wall_dt * 50));
        rotation_property_->setQuaternion(projector_node_->getOrientation());
    }
}

void MeshDisplayCustom::fixedFrameChanged()
{
    //has_new_transforms_ = true;
    context_->queueRender();
}

void MeshDisplayCustom::clear()
{
}

void MeshDisplayCustom::reset()
{
    Display::reset();
    //has_new_transforms_ = true;
}

void MeshDisplayCustom::transform(Ogre::Vector3& position, Ogre::Quaternion& orientation, const char* from_frame, const char* to_frame)
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

void MeshDisplayCustom::setRenderPanel( rviz::RenderPanel* rp )
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
