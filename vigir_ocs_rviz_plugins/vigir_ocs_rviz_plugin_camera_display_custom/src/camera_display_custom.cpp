/*
 * CameraDisplayCustom class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on the rviz image display class.
 *
 * Latest changes (12/11/2012):
 * - fixed segfault issues
 */
/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <boost/bind.hpp>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreViewport.h>

#include <tf/transform_listener.h>

#include "rviz/bit_allocator.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/render_panel.h"
#include "rviz/validate_floats.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/display_group_visibility_property.h"
#include <image_transport/camera_common.h>

#include "rviz/display.h"
#include "rviz/display_group.h"
#include "rviz/visualization_manager.h"
#include "camera_display_custom.h"
#include "rviz/load_resource.h"

#include <flor_perception_msgs/DownSampledImageRequest.h>


namespace rviz
{

const QString CameraDisplayCustom::BACKGROUND( "background" );
const QString CameraDisplayCustom::OVERLAY( "overlay" );
const QString CameraDisplayCustom::BOTH( "background and overlay" );

bool validateFloats(const sensor_msgs::CameraInfo& msg)
{
    bool valid = true;
    valid = valid && validateFloats( msg.D );
    valid = valid && validateFloats( msg.K );
    valid = valid && validateFloats( msg.R );
    valid = valid && validateFloats( msg.P );
    return valid;
}


CameraDisplayCustom::CameraDisplayCustom()
    : ImageDisplayBase()
    , texture_()
    , texture_selection_()
    , full_image_width_(800) // default parameters for atlas robot
    , full_image_height_(800)
    , full_image_binning_(8)
    , crop_x_offset_(0)
    , crop_y_offset_(0)
    , crop_width_(0)
    , crop_height_(0)
    , crop_binning_(0)
    , publish_frequency_(0.0f)
    , crop_publish_frequency_(0.0f)
    , render_panel_(NULL)
    , caminfo_tf_filter_( 0 )
    , new_caminfo_( false )
    , force_render_( false )
    , caminfo_ok_(false)
{
    image_position_property_ = new EnumProperty( "Image Rendering", BOTH,
                                                 "Render the image behind all other geometry or overlay it on top, or both.",
                                                 this, SLOT( forceRender() ));
    image_position_property_->addOption( BACKGROUND );
    image_position_property_->addOption( OVERLAY );
    image_position_property_->addOption( BOTH );

    alpha_property_ = new FloatProperty( "Overlay Alpha", 0.5,
                                         "The amount of transparency to apply to the camera image when rendered as overlay.",
                                         this, SLOT( updateAlpha() ));
    alpha_property_->setMin( 0 );
    alpha_property_->setMax( 1 );

    zoom_property_ = new FloatProperty( "Zoom Factor", 1.0,
                                        "Set a zoom factor below 1 to see a larger part of the world, above 1 to magnify the image.",
                                        this, SLOT( forceRender() ));
    zoom_property_->setMin( 0.00001 );
    zoom_property_->setMax( 100000 );

    cropped_topic_property_ = new RosTopicProperty("Cropped Image Topic", "",
                                                   QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                                   "sensor_msgs::Image topic to subscribe to.", this, SLOT( updateCroppedTopic() ));

    img_req_full_topic_property_ = new RosTopicProperty("Image Request Topic", "",
                                                        QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                                        "sensor_msgs::Image topic to subscribe to.", this, SLOT( updateImgReqTopic() ));
    img_req_cropped_topic_property_ = new RosTopicProperty("Cropped Image Request Topic", "",
                                                           QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                                           "sensor_msgs::Image topic to subscribe to.", this, SLOT( updateImgReqCroppedTopic() ));
}

CameraDisplayCustom::~CameraDisplayCustom()
{
    render_panel_->getRenderWindow()->removeListener( this );
    ImageDisplayBase::unsubscribe();
    caminfo_tf_filter_->clear();
    //delete render_panel_;
    delete bg_screen_rect_;
    delete fg_screen_rect_;
    delete screen_rect_selection_;
    delete fg_screen_rect_selection_;
    bg_scene_node_->getParentSceneNode()->removeAndDestroyChild(bg_scene_node_->getName() );
    fg_scene_node_->getParentSceneNode()->removeAndDestroyChild( fg_scene_node_->getName() );

    delete caminfo_tf_filter_;
    context_->visibilityBits()->freeBits(vis_bit_);
}

void CameraDisplayCustom::onInitialize()
{
    // caminfo_tf_filter_ = new tf::MessageFilter<sensor_msgs::CameraInfo>( *context_->getTFClient(), fixed_frame_.toStdString(),queue_size_property_->getInt(), update_nh_ );
    caminfo_tf_filter_ = new tf::MessageFilter<sensor_msgs::CameraInfo>( *context_->getTFClient(), fixed_frame_.toStdString(),queue_size_property_->getInt(), update_nh_ );
    context_->getSceneManager()->addRenderQueueListener(this);
    {
        static uint32_t count = 0;
        std::stringstream ss;
        ss << "CameraDisplayCustom" << count++;
        //img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
    }

    bg_scene_node_ = scene_node_->createChildSceneNode();
    fg_scene_node_ = scene_node_->createChildSceneNode();
    //img_scene_node_ = img_scene_manager_->getRootSceneNode()->createChildSceneNode();

    screen_rect_selection_ = NULL;
    fg_screen_rect_selection_ = NULL;
    screen_rect_highlight_ = NULL;

    // full image quad
    {
        static int count = 0;
        std::stringstream ss;
        ss << "ImageDisplayObject" << count++;

        bg_screen_rect_ = new Ogre::Rectangle2D(true);
        //   bg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 3);
        bg_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

        ss << "Material";
        bg_material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
        bg_material_->setSceneBlending( Ogre::SBT_REPLACE );
        bg_material_->setDepthWriteEnabled(false);
        bg_material_->setReceiveShadows(false);
        bg_material_->setDepthCheckEnabled(false);

        bg_material_->getTechnique(0)->setLightingEnabled(false);
        Ogre::TextureUnitState* tu = bg_material_->getTechnique(0)->getPass(0)->createTextureUnitState();
        tu->setTextureName(texture_.getTexture()->getName());
        tu->setTextureFiltering( Ogre::TFO_NONE );
        //
        tu->setAlphaOperation( Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0 );

        bg_material_->setCullingMode(Ogre::CULL_NONE);
        Ogre::AxisAlignedBox aabInf;
        aabInf.setInfinite();
        //
        bg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND+2);
        bg_screen_rect_->setBoundingBox(aabInf);
        bg_screen_rect_->setMaterial(bg_material_->getName());
        //std::cout << "Material name (full): " << material_->getName() << std::endl;
        bg_scene_node_->attachObject(bg_screen_rect_);
        bg_scene_node_->setVisible(false);



        //overlay rectangle
        fg_screen_rect_ = new Ogre::Rectangle2D(true);
        fg_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

        fg_material_ = bg_material_->clone( ss.str()+"fg" );
        fg_screen_rect_->setBoundingBox(aabInf);
        fg_screen_rect_->setMaterial(fg_material_->getName());

        fg_material_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
        fg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

        fg_scene_node_->attachObject(fg_screen_rect_);
        fg_scene_node_->setVisible(false);
    }
    updateAlpha();

    // selected area image quad
    {
        static int count = 1;
        std::stringstream ss;
        ss << "ImageDisplayObject" << count++;
        screen_rect_selection_ = new Ogre::Rectangle2D(true);
        screen_rect_selection_->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND+1);
        screen_rect_selection_->setCorners(0, 0, 0, 0);

        ss << "Material";
        material_selection_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
        material_selection_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
        material_selection_->setDepthWriteEnabled(false);
        material_selection_->setReceiveShadows(false);
        material_selection_->setDepthCheckEnabled(false);

        material_selection_->getTechnique(0)->setLightingEnabled(false);
        Ogre::TextureUnitState* tu = material_selection_->getTechnique(0)->getPass(0)->createTextureUnitState();
        tu->setTextureName(texture_selection_.getTexture()->getName());
        tu->setTextureFiltering( Ogre::TFO_NONE );

        material_selection_->setCullingMode(Ogre::CULL_NONE);
        Ogre::AxisAlignedBox aabInf;
        aabInf.setInfinite();
        screen_rect_selection_->setBoundingBox(aabInf);
        screen_rect_selection_->setMaterial(material_selection_->getName());
        //std::cout << "Material name (cropped): " << material_selection_->getName() << std::endl;
        bg_scene_node_->attachObject(screen_rect_selection_);




        ss << "ImageDisplayObject" << count-1;
        fg_screen_rect_selection_ = new Ogre::Rectangle2D(true);
        fg_screen_rect_selection_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY+1);
        fg_screen_rect_selection_->setCorners(0, 0, 0, 0);

        ss << "Material";
        fg_material_selection_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
        fg_material_selection_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
        fg_material_selection_->setDepthWriteEnabled(false);
        fg_material_selection_->setReceiveShadows(false);
        fg_material_selection_->setDepthCheckEnabled(false);

        fg_material_selection_->getTechnique(0)->setLightingEnabled(false);
        tu = fg_material_selection_->getTechnique(0)->getPass(0)->createTextureUnitState();
        tu->setTextureName(texture_selection_.getTexture()->getName());
        tu->setTextureFiltering( Ogre::TFO_NONE );

        fg_material_selection_->setCullingMode(Ogre::CULL_NONE);
        aabInf.setInfinite();
        fg_screen_rect_selection_->setBoundingBox(aabInf);
        fg_screen_rect_selection_->setMaterial(fg_material_selection_->getName());
        //std::cout << "Material name (cropped): " << material_selection_->getName() << std::endl;
        fg_scene_node_->attachObject(fg_screen_rect_selection_);
        updateSelectedAlpha(alpha_property_->getFloat());
    }

    // selected area highlight
    {
        static int count = 2;
        std::stringstream ss;
        ss << "ImageDisplayObject" << count-1;
        screen_rect_highlight_ = new Ogre::Rectangle2D(true);
        screen_rect_highlight_->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND + 1);
        screen_rect_highlight_->setCorners(0, 0, 0, 0);

        ss << "Material";
        material_highlight_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
        material_highlight_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
        material_highlight_->setDepthWriteEnabled(false);
        material_highlight_->setReceiveShadows(false);
        material_highlight_->setDepthCheckEnabled(false);
        material_highlight_->getTechnique(0)->setLightingEnabled(false);
        material_highlight_->setDiffuse(1.0,1.0,0.0,0.5);

        material_highlight_->setCullingMode(Ogre::CULL_NONE);
        Ogre::AxisAlignedBox aabInf;
        aabInf.setInfinite();
        screen_rect_highlight_->setBoundingBox(aabInf);
        screen_rect_highlight_->setMaterial(material_highlight_->getName());
        //std::cout << "Material name (highlight): " << material_highlight_->getName() << std::endl;
        bg_scene_node_->attachObject(screen_rect_highlight_);
    }

    // first create a publisher to set the parameters of the full image
    img_req_pub_full_ = n_.advertise<flor_perception_msgs::DownSampledImageRequest>( "/l_image_full/image_request", 1, true );

    // publish image request for full image - TO DO: MAKE THESE CONFIGURABLE WITH A SLOT FOR UI INTEGRATION
    publishFullImageRequest();

    // also create a publisher to set parameters of cropped image
    img_req_pub_crop_ = n_.advertise<flor_perception_msgs::DownSampledImageRequest>( "/l_image_cropped/image_request", 1, false );
    // then, subscribe to the resulting cropped image
    cropped_image_ = n_.subscribe<sensor_msgs::Image>( "/l_image_cropped/image_raw", 5, &CameraDisplayCustom::processCroppedImage, this );

    // finally, we need to subscribe to requests so that multiple clients have everything updated
    img_req_sub_crop_ = n_.subscribe<flor_perception_msgs::DownSampledImageRequest>( "/l_image_cropped/image_request", 1, &CameraDisplayCustom::processCropImageRequest, this );
    img_req_sub_full_ = n_.subscribe<flor_perception_msgs::DownSampledImageRequest>( "/l_image_full/image_request", 1, &CameraDisplayCustom::processFullImageRequest, this );

    caminfo_tf_filter_->connectInput(caminfo_sub_);
    caminfo_tf_filter_->registerCallback(boost::bind(&CameraDisplayCustom::caminfoCallback, this, _1));
}

/**
**
**
**/

void CameraDisplayCustom::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
    QString image_position = image_position_property_->getString();
    bg_scene_node_->setVisible( caminfo_ok_ && (image_position == BACKGROUND || image_position == BOTH) );
    fg_scene_node_->setVisible( caminfo_ok_ && (image_position == OVERLAY || image_position == BOTH) );
    // set view flags on all displays
    visibility_property_->update();

}

void CameraDisplayCustom::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
    bg_scene_node_->setVisible( false );
    fg_scene_node_->setVisible( false );

}


//This set up right now makes it so that the selected image comes through everything but the 
//full image doesn't cover up anything
void CameraDisplayCustom::renderQueueStarted(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation)
{
    if (queueGroupId == Ogre::RENDER_QUEUE_BACKGROUND)
    {
        Ogre::RenderSystem *rs = Ogre::Root::getSingleton().getRenderSystem();
        rs->clearFrameBuffer(Ogre::FBT_STENCIL);
        rs->setStencilCheckEnabled(true);
        rs->setStencilBufferParams(Ogre::CMPF_ALWAYS_PASS, 0x1, 0xFFFFFFFF, Ogre::SOP_REPLACE,
                                   Ogre::SOP_REPLACE,Ogre::SOP_REPLACE, false);
    }
    else if (queueGroupId == Ogre::RENDER_QUEUE_BACKGROUND+1 || queueGroupId == Ogre::RENDER_QUEUE_OVERLAY+1)
    {
        Ogre::RenderSystem *rs = Ogre::Root::getSingleton().getRenderSystem();
        rs->setStencilCheckEnabled(true);
        rs->setStencilBufferParams(Ogre::CMPF_EQUAL, 0x1, 0xFFFFFFFF, Ogre::SOP_REPLACE,
                                   Ogre::SOP_REPLACE,Ogre::SOP_REPLACE, false);
    }
    else if (queueGroupId != Ogre::RENDER_QUEUE_BACKGROUND+2 && queueGroupId != Ogre::RENDER_QUEUE_OVERLAY)
    {
        Ogre::RenderSystem *rs = Ogre::Root::getSingleton().getRenderSystem();
        rs->setStencilCheckEnabled(true);
        rs->setStencilBufferParams(Ogre::CMPF_ALWAYS_PASS, 0x1, 0xFFFFFFFF, Ogre::SOP_KEEP,
                                   Ogre::SOP_KEEP,Ogre::SOP_KEEP, false);
    }
    else
    {
        Ogre::RenderSystem *rs = Ogre::Root::getSingleton().getRenderSystem();
        rs->setStencilCheckEnabled(true);
        rs->setStencilBufferParams(Ogre::CMPF_NOT_EQUAL, 0x1,  0xFFFFFFFF, Ogre::SOP_KEEP,
                                   Ogre::SOP_KEEP, Ogre::SOP_KEEP, false);
    }

}

void CameraDisplayCustom::renderQueueEnded(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& repeatThisInvocation)
{
    if (queueGroupId == Ogre::RENDER_QUEUE_OVERLAY+1)  {
        Ogre::RenderSystem *rs = Ogre::Root::getSingleton().getRenderSystem();
        rs->setStencilCheckEnabled(false);
        rs->setStencilBufferParams();
    }
}

void CameraDisplayCustom::onEnable()
{
    if ( (!isEnabled()) || (topic_property_->getTopicStd().empty()) )
    {
        if((topic_property_->getTopicStd().empty()))
        {
            //std::cout<<"The error is the topic std" <<std::endl;
        }
        return;
    }

    /**
    * These next two lines make it so that "No Image" appears for some reason.
    */
    std::string target_frame = fixed_frame_.toStdString();

    ImageDisplayBase::enableTFFilter(target_frame);
    ImageDisplayBase::subscribe();

    std::string topic = topic_property_->getTopicStd();
    std::string caminfo_topic = image_transport::getCameraInfoTopic(topic_property_->getTopicStd());

    try
    {
        caminfo_sub_.subscribe( update_nh_, caminfo_topic, 1 );
        // std::cout<<"The subscription happens"<<std::endl;
        setStatus( StatusProperty::Ok, "Camera Info", "OK" );
    }
    catch( ros::Exception& e )
    {
        setStatus( StatusProperty::Error, "Camera Info", QString( "Error subscribing: ") + e.what() );
    }

    if(render_panel_)
    {
        render_panel_->getRenderWindow()->setActive(true);
        //This may be wrong

    }
}

void CameraDisplayCustom::onDisable()
{
    if(render_panel_)
        render_panel_->getRenderWindow()->setActive(false);
    ImageDisplayBase::unsubscribe();
    caminfo_sub_.unsubscribe();
    clear();
}

void CameraDisplayCustom::clear()
{
    texture_.clear();
    force_render_ = true;
    context_->queueRender();
    texture_selection_.clear();

    new_caminfo_ = false;
    current_caminfo_.reset();
    if(render_panel_)
    {
        if( render_panel_->getCamera() )
        {
            render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
        }
    }
}

void CameraDisplayCustom::update( float wall_dt, float ros_dt )
{
    if(render_panel_)
    {
        try
        {
            if(texture_.update()||texture_selection_.update()||force_render_)
            {

                caminfo_ok_ = updateCamera();
                force_render_ = false;
            }
        }
        catch( UnsupportedImageEncoding& e )
        {
            setStatus(StatusProperty::Error, "Image", e.what());
        }

        render_panel_->getRenderWindow()->update();
    }
}


bool CameraDisplayCustom::updateCamera()
{

    sensor_msgs::CameraInfo::ConstPtr info;
    sensor_msgs::Image::ConstPtr image;
    {
        boost::mutex::scoped_lock lock( caminfo_mutex_ );

        info = current_caminfo_;
        image = texture_.getImage();
    }
    if(!info || !image)
    {

        //std::cout<<"Still has an issue"<<std::endl;
        return false;
    }

    if( !validateFloats( *info ))
    {
        setStatus( StatusProperty::Error, "Camera Info", "Contains invalid floating point values (nans or infs)" );
        return false;
    }

    Q_EMIT updateFrameID(info->header.frame_id);

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    //image->header.frame_id (this stuff is grabbed from a sensor_msg)
    context_->getFrameManager()->getTransform( image->header.frame_id, image->header.stamp, position, orientation );

    // convert vision (Z-forward) frame to ogre frame (Z-out)
    orientation = orientation * Ogre::Quaternion( Ogre::Degree( 180 ), Ogre::Vector3::UNIT_X );

    double fx = info->P[0];
    double fy = info->P[5];
    //make sure the aspect ratio of the image is preserved
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    float img_width = info->width;
    float img_height = info->height;
    float zoom_x = zoom_property_->getFloat();
    float zoom_y = zoom_x;
    // Preserve aspect ratio
    if( win_width != 0 && win_height != 0 )
    {
        float img_aspect = (img_width/fx) / (img_height/fy);
        float win_aspect = win_width / win_height;

        if ( img_aspect > win_aspect )
        {
            zoom_y = zoom_y / img_aspect * win_aspect;
        }
        else
        {
            zoom_x = zoom_x / win_aspect * img_aspect;
        }
    }

    // Add the camera's translation relative to the left camera (from P[3]);
    double tx = -1 * (info->P[3] / fx);
    Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
    position = position + (right * tx);

    double ty = -1 * (info->P[7] / fy);
    Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
    position = position + (down * ty);

    if( !validateFloats( position ))
    {
        setStatus( StatusProperty::Error, "Camera Info", "CameraInfo/P resulted in an invalid position calculation (nans or infs)" );
        return false;
    }

    render_panel_->getCamera()->setPosition( position );
    render_panel_->getCamera()->setOrientation( orientation );

    // calculate the projection matrix
    double cx = info->P[2];
    double cy = info->P[6];

    double far_plane = 100;
    double near_plane = 0.01;

    Ogre::Matrix4 proj_matrix;
    proj_matrix = Ogre::Matrix4::ZERO;

    proj_matrix[0][0]= 2.0 * fx/img_width * zoom_x;
    proj_matrix[1][1]= 2.0 * fy/img_height * zoom_y;

    proj_matrix[0][2]= 2.0 * (0.5 - cx/img_width) * zoom_x;
    proj_matrix[1][2]= 2.0 * (cy/img_height - 0.5) * zoom_y;

    proj_matrix[2][2]= -(far_plane+near_plane) / (far_plane-near_plane);
    proj_matrix[2][3]= -2.0*far_plane*near_plane / (far_plane-near_plane);

    proj_matrix[3][2]= -1;

    render_panel_->getCamera()->setCustomProjectionMatrix( true, proj_matrix );

    setStatus( StatusProperty::Ok, "Camera Info", "OK" );

#if 0
    static Axes* debug_axes = new Axes(scene_manager_, 0, 0.2, 0.01);
    debug_axes->setPosition(position);
    debug_axes->setOrientation(orientation);
#endif


    if ( img_width != 0 && img_height != 0 && win_width !=0 && win_height != 0 )
    {
        float img_aspect = img_width / img_height;
        float win_aspect = win_width / win_height;

        // calculate size of 3 pixels in scene CS for selection highlight
        float padding_x = 3.0f*2.0f/win_width;
        float padding_y = 3.0f*2.0f/win_height;

        if ( img_aspect > win_aspect )
        {
            //The camera_display file deals with zoom here.
            bg_screen_rect_->setCorners(-1.0f, 1.0f * win_aspect/img_aspect, 1.0f, -1.0f * win_aspect/img_aspect, false);
            //This will most likely change
            fg_screen_rect_->setCorners(-1.0f, 1.0f * win_aspect/img_aspect, 1.0f, -1.0f * win_aspect/img_aspect, false);
            /**
             * //adjust the image rectangles to fit the zoom & aspect ratio
             *  bg_screen_rect_->setCorners( -1.0f*zoom_x, 1.0f*zoom_y, 1.0f*zoom_x, -1.0f*zoom_y );
             * fg_screen_rect_->setCorners( -1.0f*zoom_x, 1.0f*zoom_y, 1.0f*zoom_x, -1.0f*zoom_y );
             **/
            // calculate full image rectangle dimensions in window coordinates
            rect_dim_x1_ = 0;
            rect_dim_x2_ = win_width;
            rect_dim_y1_ = (win_height - (win_height * win_aspect/img_aspect)) / 2.0f;
            rect_dim_y2_ = win_height - rect_dim_y1_;

            // calculate selection rectangle position
            if(screen_rect_selection_ != NULL)
            {
                // full image size: -1.0f, 1.0f * win_aspect/img_aspect, 1.0f, -1.0f * win_aspect/img_aspect
                float x1 = ((2.0f * crop_x_offset_) / full_image_width_) - 1.0f;
                float x2 = ((2.0f * (crop_x_offset_+crop_width_)) / full_image_width_) - 1.0f;
                float y1 = -(((2.0f * win_aspect/img_aspect) * crop_y_offset_) / full_image_height_) + ((1.0f * win_aspect/img_aspect));
                float y2 = -(((2.0f * win_aspect/img_aspect) * (crop_y_offset_+crop_height_)) / full_image_height_) + ((1.0f * win_aspect/img_aspect));
                screen_rect_selection_->setCorners(x1,y1,x2,y2, false);
                fg_screen_rect_selection_->setCorners(x1,y1,x2,y2, false);
                screen_rect_highlight_->setCorners(x1-padding_x,y1+padding_y,x2+padding_x,y2-padding_y, false);
                //std::cout << "Select Window: " << x1 << ", " << y1 << " -> " << x2 << ", " << y2 << std::endl;
            }
        }
        else
        {
            bg_screen_rect_->setCorners(-1.0f * img_aspect/win_aspect, 1.0f, 1.0f * img_aspect/win_aspect, -1.0f, false);
            //this will most likely change
            fg_screen_rect_->setCorners(-1.0f * img_aspect/win_aspect, 1.0f, 1.0f * img_aspect/win_aspect, -1.0f, false);
            // calculate full image rectangle dimensions in window coordinates
            rect_dim_x1_ = (win_width - (win_width * img_aspect/win_aspect)) / 2.0f;
            rect_dim_x2_ = win_width - rect_dim_x1_;
            rect_dim_y1_ = 0;
            rect_dim_y2_ = win_height;

            // calculate selection rectangle position
            if(screen_rect_selection_ != NULL)
            {
                // full image size: -1.0f, 1.0f * win_aspect/img_aspect, 1.0f, -1.0f * win_aspect/img_aspect
                float x1 = (((2.0f * img_aspect/win_aspect) * crop_x_offset_) / full_image_width_) - (1.0f * img_aspect/win_aspect);
                float x2 = (((2.0f * img_aspect/win_aspect) * (crop_x_offset_+crop_width_)) / full_image_width_) - (1.0f * img_aspect/win_aspect);
                float y1 = -((2.0f * crop_y_offset_) / full_image_height_) + 1.0f;
                float y2 = -((2.0f * (crop_y_offset_+crop_height_)) / full_image_height_) + 1.0f;
                screen_rect_selection_->setCorners(x1,y1,x2,y2, false);
                fg_screen_rect_selection_->setCorners(x1,y1,x2,y2, false);
                screen_rect_highlight_->setCorners(x1-padding_x,y1+padding_y,x2+padding_x,y2-padding_y, false);
                //std::cout << "Select Window: " << x1 << ", " << y1 << " -> " << x2 << ", " << y2 << std::endl;
            }
        }
    }

    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    bg_screen_rect_->setBoundingBox( aabInf );
    fg_screen_rect_->setBoundingBox( aabInf );

    setStatus( StatusProperty::Ok, "Time", "ok" );
    setStatus( StatusProperty::Ok, "Camera Info", "ok" );

    return true;
}

void CameraDisplayCustom::reset()
{
    ImageDisplayBase::reset();
    clear();
}

/* This is called by incomingMessage(). */
void CameraDisplayCustom::processMessage(const sensor_msgs::Image::ConstPtr& msg)
{
    texture_.addMessage(msg);
}

void CameraDisplayCustom::caminfoCallback( const sensor_msgs::CameraInfo::ConstPtr& msg )
{
    //std::cout<<"This is called at some point"<<std::endl;
    boost::mutex::scoped_lock lock( caminfo_mutex_ );
    current_caminfo_ = msg;
    new_caminfo_ = true;
}


void CameraDisplayCustom::processCroppedImage(const sensor_msgs::Image::ConstPtr& msg)
{
    texture_selection_.addMessage(msg);
}

void CameraDisplayCustom::processFullImageRequest(const flor_perception_msgs::DownSampledImageRequest::ConstPtr& msg)
{
    full_image_binning_ = msg->binning_x;
    full_image_width_ = msg->roi.width;
    full_image_height_ = msg->roi.height;
    publish_frequency_ = msg->publish_frequency;
}

void CameraDisplayCustom::processCropImageRequest(const flor_perception_msgs::DownSampledImageRequest::ConstPtr& msg)
{
    crop_binning_ = msg->binning_x;
    crop_width_ = msg->roi.width;
    crop_height_ = msg->roi.height;
    crop_x_offset_ = msg->roi.x_offset;
    crop_y_offset_ = msg->roi.y_offset;
    crop_publish_frequency_ = msg->publish_frequency;
}

void CameraDisplayCustom::setRenderPanel( RenderPanel* rp )
{
    render_panel_ = rp;
    render_panel_->getRenderWindow()->addListener( this );

    render_panel_->getRenderWindow()->setAutoUpdated(false);
    render_panel_->getRenderWindow()->setActive( false );

    render_panel_->setAutoRender(false);
    render_panel_->setOverlaysEnabled(false);
    render_panel_->getCamera()->setNearClipDistance( 0.01f );
}

void CameraDisplayCustom::selectionProcessed( int x1, int y1, int x2, int y2 )
{
    std::cout << "Select Window: " << x1 << ", " << y1 << " -> " << x2 << ", " << y2 << std::endl;
    //std::cout << "   full image rect: " << rect_dim_x1_ << ", " << rect_dim_y1_ << " -> " << rect_dim_x2_ << ", " << rect_dim_y2_ << std::endl;
    //std::cout << "   window dimensions: " << render_panel_->width() << ", " << render_panel_->height() << std::endl;

    // make sure selection is within image rect coordinates
    float minx = std::min(x1, x2);
    float _x1 = (minx < rect_dim_x1_ ? rect_dim_x1_ : (minx > rect_dim_x2_ ? rect_dim_x2_ : minx));
    float maxx = std::max(x1, x2);
    float _x2 = (maxx < rect_dim_x1_ ? rect_dim_x1_ : (maxx > rect_dim_x2_ ? rect_dim_x2_ : maxx));
    float miny = std::min(y1, y2);
    float _y1 = (miny < rect_dim_y1_ ? rect_dim_y1_ : (miny > rect_dim_y2_ ? rect_dim_y2_ : miny));
    float maxy = std::max(y1, y2);
    float _y2 = (maxy < rect_dim_y1_ ? rect_dim_y1_ : (maxy > rect_dim_y2_ ? rect_dim_y2_ : maxy));
    //std::cout << "   corrected window: " << _x1 << ", " << _y1 << " -> " << _x2 << ", " << _y2 << std::endl;

    // calculate image selection box dimensions -> texture coordinates
    crop_x_offset_ = (_x1-rect_dim_x1_) * full_image_width_ / (rect_dim_x2_-rect_dim_x1_);
    crop_y_offset_ = (_y1-rect_dim_y1_) * full_image_height_ / (rect_dim_y2_-rect_dim_y1_);
    // and size
    crop_width_  = ((_x2-_x1) * full_image_width_) / (rect_dim_x2_-rect_dim_x1_);
    crop_height_ = ((_y2-_y1) * full_image_height_) / (rect_dim_y2_-rect_dim_y1_);

    // create image request message
    publishCropImageRequest();

    std::cout << "   image selection rect: " << crop_x_offset_ << ", " << crop_y_offset_ << " -> " << (crop_x_offset_+crop_width_) << ", " << (crop_y_offset_+crop_height_) << std::endl;
}

void CameraDisplayCustom::changeFullImageResolution( int t )
{
    std::cout << "Full image resolution changed:" << t << std::endl;
    switch( t )
    {
    case IMAGE_RESOLUTION_FULL:
    {
        full_image_binning_ = 1;
    }
        break;
    case IMAGE_RESOLUTION_2:
    {
        full_image_binning_ = 2;
    }
        break;
    case IMAGE_RESOLUTION_4:
    {
        full_image_binning_ = 4;
    }
        break;
    case IMAGE_RESOLUTION_8:
    {
        full_image_binning_ = 8;
    }
        break;
    case IMAGE_RESOLUTION_16:
    {
        full_image_binning_ = 16;
    }
        break;
    }

    publishFullImageRequest();
}

void CameraDisplayCustom::changeCropImageResolution( int t )
{
    std::cout << "Crop image resolution changed:" << t << std::endl;
    switch( t )
    {
    case IMAGE_RESOLUTION_FULL:
    {
        crop_binning_ = 1;
    }
        break;
    case IMAGE_RESOLUTION_2:
    {
        crop_binning_ = 2;
    }
        break;
    case IMAGE_RESOLUTION_4:
    {
        crop_binning_ = 4;
    }
        break;
    case IMAGE_RESOLUTION_8:
    {
        crop_binning_ = 8;
    }
        break;
    case IMAGE_RESOLUTION_16:
    {
        crop_binning_ = 16;
    }
        break;
    }

    publishCropImageRequest();
}

void CameraDisplayCustom::changeCameraSpeed( int t )
{
    std::cout << "Camera speed changed:" << t << std::endl;//(15.0f/(float)pow(3,t)) << std::endl;

    publish_frequency_ = t;//15.0f/(float)pow(3,t); // 15 or whatever the max fps is

    publishFullImageRequest();
    //publishCropImageRequest();
}

void CameraDisplayCustom::changeCropCameraSpeed( int t )
{
    std::cout << "Camera speed changed:" << t << std::endl;//(15.0f/(float)pow(3,t)) << std::endl;

    crop_publish_frequency_ = t;//15.0f/(float)pow(3,t); // 15 or whatever the max fps is

    //publishFullImageRequest();
    publishCropImageRequest();
}


void CameraDisplayCustom::publishCropImageRequest()
{
    flor_perception_msgs::DownSampledImageRequest cmd;

    cmd.binning_x = crop_binning_;
    cmd.binning_y = crop_binning_;
    cmd.roi.width = crop_width_;
    cmd.roi.height = crop_height_;
    cmd.roi.x_offset = crop_x_offset_;
    cmd.roi.y_offset = crop_y_offset_;
    if(crop_publish_frequency_ == 0.0f)
        cmd.mode = 2;
    else
        cmd.mode = 1;
    cmd.publish_frequency = crop_publish_frequency_;

    // publish image request for cropped image
    img_req_pub_crop_.publish( cmd );
}

void CameraDisplayCustom::publishFullImageRequest()
{
    flor_perception_msgs::DownSampledImageRequest cmd;

    cmd.binning_x = full_image_binning_;
    cmd.binning_y = full_image_binning_;
    cmd.roi.width = full_image_width_;
    cmd.roi.height = full_image_height_;
    cmd.roi.x_offset = 0;
    cmd.roi.y_offset = 0;
    if(publish_frequency_ == 0.0f)
        cmd.mode = 2;
    else
        cmd.mode = 1;
    cmd.publish_frequency = publish_frequency_;

    // publish image request for full image
    img_req_pub_full_.publish( cmd );

}



void CameraDisplayCustom::forceRender()
{
    // std::cout<<"this is forced to render"<<std::endl;
    force_render_ = true;
    context_->queueRender();
}

void CameraDisplayCustom::updateAlpha()
{
    float alpha = alpha_property_->getFloat();

    Ogre::Pass* pass = fg_material_->getTechnique( 0 )->getPass( 0 );
    if( pass->getNumTextureUnitStates() > 0 )
    {
        Ogre::TextureUnitState* tex_unit = pass->getTextureUnitState( 0 );
        tex_unit->setAlphaOperation( Ogre::LBX_MODULATE, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha );
    }
    else
    {
        fg_material_->setAmbient( Ogre::ColourValue( 0.0f, 1.0f, 1.0f, alpha ));
        fg_material_->setDiffuse( Ogre::ColourValue( 0.0f, 1.0f, 1.0f, alpha ));
    }

    force_render_ = true;
    if(fg_screen_rect_selection_ != NULL)
    {
        updateSelectedAlpha(alpha);
    }
    context_->queueRender();
}

/**
* This is used to change the alpha of the selected area when the image
* is overlaying the camera display information.
**/
void CameraDisplayCustom::updateSelectedAlpha(float selectedAlpha)
{
    float alpha = selectedAlpha;

    Ogre::Pass* pass =  fg_material_selection_->getTechnique( 0 )->getPass( 0 );
    if( pass->getNumTextureUnitStates() > 0 )
    {
        Ogre::TextureUnitState* tex_unit = pass->getTextureUnitState( 0 );
        tex_unit->setAlphaOperation( Ogre::LBX_MODULATE, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha );
    }
    else
    {
        fg_material_selection_->setAmbient( Ogre::ColourValue( 0.0f, 1.0f, 1.0f, alpha ));
        fg_material_selection_->setDiffuse( Ogre::ColourValue( 0.0f, 1.0f, 1.0f, alpha ));
    }
    //  std::cout<<"It should be adjusting transparency"<<std::endl;
    force_render_ = true;
    context_->queueRender();
}


void CameraDisplayCustom::updateQueueSize()
{
    caminfo_tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
    ImageDisplayBase::updateQueueSize();
}


void CameraDisplayCustom::fixedFrameChanged()
{
    std::string targetFrame = fixed_frame_.toStdString();
    //caminfo_tf_filter_->setTargetFrame(targetFrame);
    caminfo_tf_filter_->setTargetFrame(fixed_frame_.toStdString());
    //  texture_.setFrame(fixed_frame_, context_->getTFClient());

    ImageDisplayBase::fixedFrameChanged();
}

void CameraDisplayCustom::setup()
{
    DisplayGroup* display = context_->getRootDisplayGroup();
    Display* theDisplay = display->getDisplayAt(0);
    vis_bit_ = context_->visibilityBits()->allocBit();
    //vis_bit_ = manager_->visibilityBits()->allocBit();
    render_panel_->getViewport()->setVisibilityMask( vis_bit_ );

    visibility_property_ = new DisplayGroupVisibilityProperty(
                vis_bit_,context_->getRootDisplayGroup() , this, "Visibility", true,
                "Changes the visibility of other Displays in the camera view.");

    visibility_property_->update();
    visibility_property_->setIcon( loadPixmap("package://rviz/icons/visibility.svg",true) );

    this->addChild( visibility_property_, 0 );
}


void CameraDisplayCustom::setAlpha(float newAlpha)
{
    alpha_property_->setFloat(newAlpha);
    updateAlpha();
}


void CameraDisplayCustom::setZoom(float newZoom)
{
    zoom_property_->setFloat(newZoom);
}


void CameraDisplayCustom::closeSelected()
{
    selectionProcessed(0,0,0,0);
}

void CameraDisplayCustom::updateCroppedTopic()
{
    std::cout << "new cropped image topic: " << cropped_topic_property_->getTopicStd() << std::endl;

    if(!cropped_topic_property_->getTopicStd().empty())
    {
        // unsubscribe
        cropped_image_.shutdown();
        // then, subscribe to the resulting cropped image
        cropped_image_ = n_.subscribe<sensor_msgs::Image>( cropped_topic_property_->getTopicStd(), 5, &CameraDisplayCustom::processCroppedImage, this );
    }
}

void CameraDisplayCustom::updateImgReqTopic()
{
    std::cout << "new image request topic: " << img_req_full_topic_property_->getTopicStd() << std::endl;

    if(!img_req_full_topic_property_->getTopicStd().empty())
    {
        img_req_pub_full_.shutdown();
        img_req_pub_full_ = n_.advertise<flor_perception_msgs::DownSampledImageRequest>( img_req_full_topic_property_->getTopicStd(), 1, true );

        // publish image request for full image - TO DO: MAKE THESE CONFIGURABLE WITH A SLOT FOR UI INTEGRATION
        publishFullImageRequest();

        img_req_sub_full_.shutdown();
        // finally, we need to subscribe to requests so that multiple clients have everything updated
        img_req_sub_full_ = n_.subscribe<flor_perception_msgs::DownSampledImageRequest>( img_req_full_topic_property_->getTopicStd(), 1, &CameraDisplayCustom::processFullImageRequest, this );
    }
}

void CameraDisplayCustom::updateImgReqCroppedTopic()
{
    std::cout << "new crop request topic: " << img_req_cropped_topic_property_->getTopicStd() << std::endl;

    if(!img_req_cropped_topic_property_->getTopicStd().empty())
    {
        img_req_pub_crop_.shutdown();
        // also create a publisher to set parameters of cropped image
        img_req_pub_crop_ = n_.advertise<flor_perception_msgs::DownSampledImageRequest>( img_req_cropped_topic_property_->getTopicStd(), 1, false );

        img_req_sub_crop_.shutdown();
        // finally, we need to subscribe to requests so that multiple clients have everything updated
        img_req_sub_crop_ = n_.subscribe<flor_perception_msgs::DownSampledImageRequest>( img_req_cropped_topic_property_->getTopicStd(), 1, &CameraDisplayCustom::processCropImageRequest, this );
    }
}

} // namespace rviz

