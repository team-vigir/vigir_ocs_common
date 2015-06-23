/* 
 * ImageDisplayCustom class implementation.
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

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/render_panel.h"
#include "rviz/validate_floats.h"

#include "image_display_custom.h"

#include <vigir_perception_msgs/DownSampledImageRequest.h>

namespace rviz
{

ImageDisplayCustom::ImageDisplayCustom()
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
    , publish_frequency_(15.0f)
    , render_panel_(NULL)
{
}

void ImageDisplayCustom::onInitialize()
{
    {
        static uint32_t count = 0;
        std::stringstream ss;
        ss << "ImageDisplayCustom" << count++;
        //img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
    }

    //img_scene_node_ = img_scene_manager_->getRootSceneNode()->createChildSceneNode();

//    screen_rect_selection_ = NULL;
//    screen_rect_highlight_ = NULL;

    // full image quad
    {
        static int count = 0;
        std::stringstream ss;
        ss << "ImageDisplayObject" << count++;

        screen_rect_ = new Ogre::Rectangle2D(true);
        screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 3);
        screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

        ss << "Material";
        material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
        material_->setSceneBlending( Ogre::SBT_REPLACE );
        material_->setDepthWriteEnabled(false);
        material_->setReceiveShadows(false);
        material_->setDepthCheckEnabled(false);

        material_->getTechnique(0)->setLightingEnabled(false);
        Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
        tu->setTextureName(texture_.getTexture()->getName());
        tu->setTextureFiltering( Ogre::TFO_NONE );

        material_->setCullingMode(Ogre::CULL_NONE);
        Ogre::AxisAlignedBox aabInf;
        aabInf.setInfinite();
        screen_rect_->setBoundingBox(aabInf);
        screen_rect_->setMaterial(material_->getName());
        //std::cout << "Material name (full): " << material_->getName() << std::endl;
        scene_node_->attachObject(screen_rect_);
    }

//    // selected area image quad
//    {
//        static int count = 1;
//        std::stringstream ss;
//        ss << "ImageDisplayObject" << count++;
//        screen_rect_selection_ = new Ogre::Rectangle2D(true);
//        screen_rect_selection_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
//        screen_rect_selection_->setCorners(0, 0, 0, 0);

//        ss << "Material";
//        material_selection_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
//        material_selection_->setSceneBlending( Ogre::SBT_REPLACE );
//        material_selection_->setDepthWriteEnabled(false);
//        material_selection_->setReceiveShadows(false);
//        material_selection_->setDepthCheckEnabled(false);

//        material_selection_->getTechnique(0)->setLightingEnabled(false);
//        Ogre::TextureUnitState* tu = material_selection_->getTechnique(0)->getPass(0)->createTextureUnitState();
//        tu->setTextureName(texture_selection_.getTexture()->getName());
//        tu->setTextureFiltering( Ogre::TFO_NONE );

//        material_selection_->setCullingMode(Ogre::CULL_NONE);
//        Ogre::AxisAlignedBox aabInf;
//        aabInf.setInfinite();
//        screen_rect_selection_->setBoundingBox(aabInf);
//        screen_rect_selection_->setMaterial(material_selection_->getName());
//        //std::cout << "Material name (cropped): " << material_selection_->getName() << std::endl;
//        scene_node_->attachObject(screen_rect_selection_);
//    }

    // selected area highlight
//    {
//        static int count = 2;
//        std::stringstream ss;
//        ss << "ImageDisplayObject" << count++;
//        screen_rect_highlight_ = new Ogre::Rectangle2D(true);
//        screen_rect_highlight_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 2);
//        screen_rect_highlight_->setCorners(0, 0, 0, 0);

//        ss << "Material";
//        material_highlight_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
//        material_highlight_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
//        material_highlight_->setDepthWriteEnabled(false);
//        material_highlight_->setReceiveShadows(false);
//        material_highlight_->setDepthCheckEnabled(false);
//        material_highlight_->getTechnique(0)->setLightingEnabled(false);
//        material_highlight_->setDiffuse(1.0,1.0,0.0,0.5);

//        material_highlight_->setCullingMode(Ogre::CULL_NONE);
//        Ogre::AxisAlignedBox aabInf;
//        aabInf.setInfinite();
//        screen_rect_highlight_->setBoundingBox(aabInf);
//        screen_rect_highlight_->setMaterial(material_highlight_->getName());
//        //std::cout << "Material name (highlight): " << material_highlight_->getName() << std::endl;
//        scene_node_->attachObject(screen_rect_highlight_);
//    }

    // first create a publisher to set the parameters of the full image
    img_req_pub_full_ = n_.advertise<vigir_perception_msgs::DownSampledImageRequest>( "/l_image_full/image_request", 1, true );

    // publish image request for full image - TO DO: MAKE THESE CONFIGURABLE WITH A SLOT FOR UI INTEGRATION
//    publishFullImageRequest();

    // also create a publisher to set parameters of cropped image
//    img_req_pub_crop_ = n_.advertise<vigir_perception_msgs::DownSampledImageRequest>( "/l_image_cropped/image_request", 1, false );
    // then, subscribe to the resulting cropped image
//    cropped_image_ = n_.subscribe<sensor_msgs::Image>( "/l_image_cropped/image_raw", 5, &ImageDisplayCustom::processCroppedImage, this );

    // finally, we need to subscribe to requests so that multiple clients have everything updated
//    img_req_sub_crop_ = n_.subscribe<vigir_perception_msgs::DownSampledImageRequest>( "/l_image_cropped/image_request", 1, &ImageDisplayCustom::processCropImageRequest, this );
//    img_req_sub_full_ = n_.subscribe<vigir_perception_msgs::DownSampledImageRequest>( "/l_image_full/image_request", 1, &ImageDisplayCustom::processFullImageRequest, this );
}

ImageDisplayCustom::~ImageDisplayCustom()
{
    ImageDisplayBase::unsubscribe();
    //delete render_panel_;
    delete screen_rect_;
//    delete screen_rect_selection_;
    scene_node_->getParentSceneNode()->removeAndDestroyChild( scene_node_->getName() );
}

void ImageDisplayCustom::onEnable()
{
    ImageDisplayBase::subscribe();
    if(render_panel_)
        render_panel_->getRenderWindow()->setActive(true);
}

void ImageDisplayCustom::onDisable()
{
    if(render_panel_)
        render_panel_->getRenderWindow()->setActive(false);
    ImageDisplayBase::unsubscribe();
    clear();
}

void ImageDisplayCustom::clear()
{
    texture_.clear();
    texture_selection_.clear();

    if(render_panel_)
    {
        if( render_panel_->getCamera() )
        {
            render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
        }
    }
}

void ImageDisplayCustom::update( float wall_dt, float ros_dt )
{
    if(render_panel_)
    {
        try
        {
            if(texture_.update())
            {
//            texture_selection_.update();

                //make sure the aspect ratio of the image is preserved
                float win_width = render_panel_->width();
                float win_height = render_panel_->height();

                float img_width = texture_.getWidth();
                float img_height = texture_.getHeight();

                if ( img_width != 0 && img_height != 0 && win_width !=0 && win_height != 0 )
                {
                    float img_aspect = img_width / img_height;
                    float win_aspect = win_width / win_height;

                    // calculate size of 3 pixels in scene CS for selection highlight
                    float padding_x = 3.0f*2.0f/win_width;
                    float padding_y = 3.0f*2.0f/win_height;

                    if ( img_aspect > win_aspect )
                    {
                        screen_rect_->setCorners(-1.0f, 1.0f * win_aspect/img_aspect, 1.0f, -1.0f * win_aspect/img_aspect, false);
                        // calculate full image rectangle dimensions in window coordinates
                        rect_dim_x1_ = 0;
                        rect_dim_x2_ = win_width;
                        rect_dim_y1_ = (win_height - (win_height * win_aspect/img_aspect)) / 2.0f;
                        rect_dim_y2_ = win_height - rect_dim_y1_;

                    // calculate selection rectangle position
//                        if(screen_rect_selection_ != NULL)
//                        {
//                            // full image size: -1.0f, 1.0f * win_aspect/img_aspect, 1.0f, -1.0f * win_aspect/img_aspect
//                            float x1 = ((2.0f * crop_x_offset_) / full_image_width_) - 1.0f;
//                            float x2 = ((2.0f * (crop_x_offset_+crop_width_)) / full_image_width_) - 1.0f;
//                            float y1 = -(((2.0f * win_aspect/img_aspect) * crop_y_offset_) / full_image_height_) + ((1.0f * win_aspect/img_aspect));
//                            float y2 = -(((2.0f * win_aspect/img_aspect) * (crop_y_offset_+crop_height_)) / full_image_height_) + ((1.0f * win_aspect/img_aspect));
//                            screen_rect_selection_->setCorners(x1,y1,x2,y2, false);
//                            screen_rect_highlight_->setCorners(x1-padding_x,y1+padding_y,x2+padding_x,y2-padding_y, false);
//                            //std::cout << "Select Window: " << x1 << ", " << y1 << " -> " << x2 << ", " << y2 << std::endl;
//                        }
                    }
                    else
                    {
                        screen_rect_->setCorners(-1.0f * img_aspect/win_aspect, 1.0f, 1.0f * img_aspect/win_aspect, -1.0f, false);
                        // calculate full image rectangle dimensions in window coordinates
                        rect_dim_x1_ = (win_width - (win_width * img_aspect/win_aspect)) / 2.0f;
                        rect_dim_x2_ = win_width - rect_dim_x1_;
                        rect_dim_y1_ = 0;
                        rect_dim_y2_ = win_height;

                    // calculate selection rectangle position
//                        if(screen_rect_selection_ != NULL)
//                        {
//                            // full image size: -1.0f, 1.0f * win_aspect/img_aspect, 1.0f, -1.0f * win_aspect/img_aspect
//                            float x1 = (((2.0f * img_aspect/win_aspect) * crop_x_offset_) / full_image_width_) - (1.0f * img_aspect/win_aspect);
//                            float x2 = (((2.0f * img_aspect/win_aspect) * (crop_x_offset_+crop_width_)) / full_image_width_) - (1.0f * img_aspect/win_aspect);
//                            float y1 = -((2.0f * crop_y_offset_) / full_image_height_) + 1.0f;
//                            float y2 = -((2.0f * (crop_y_offset_+crop_height_)) / full_image_height_) + 1.0f;
//                            screen_rect_selection_->setCorners(x1,y1,x2,y2, false);
//                            screen_rect_highlight_->setCorners(x1-padding_x,y1+padding_y,x2+padding_x,y2-padding_y, false);
//                            //std::cout << "Select Window: " << x1 << ", " << y1 << " -> " << x2 << ", " << y2 << std::endl;
//                        }
                    }
                }

                render_panel_->getRenderWindow()->update();
            }

            //std::cout << "CUSTOM UPDATE LOOP" << std::endl;
        }
        catch( UnsupportedImageEncoding& e )
        {
            setStatus(StatusProperty::Error, "Image", e.what());
        }
    }
}

void ImageDisplayCustom::reset()
{
    ImageDisplayBase::reset();
    clear();
}

/* This is called by incomingMessage(). */
void ImageDisplayCustom::processMessage(const sensor_msgs::Image::ConstPtr& msg)
{
    texture_.addMessage(msg);
}

void ImageDisplayCustom::processCroppedImage(const sensor_msgs::Image::ConstPtr& msg)
{
    texture_selection_.addMessage(msg);
}

void ImageDisplayCustom::processFullImageRequest(const vigir_perception_msgs::DownSampledImageRequest::ConstPtr& msg)
{
    full_image_binning_ = msg->binning_x;
    full_image_width_ = msg->roi.width;
    full_image_height_ = msg->roi.height;
    publish_frequency_ = msg->publish_frequency;
}

void ImageDisplayCustom::processCropImageRequest(const vigir_perception_msgs::DownSampledImageRequest::ConstPtr& msg)
{
    crop_binning_ = msg->binning_x;
    crop_width_ = msg->roi.width;
    crop_height_ = msg->roi.height;
    crop_x_offset_ = msg->roi.x_offset;
    crop_y_offset_ = msg->roi.y_offset;
    publish_frequency_ = msg->publish_frequency;
}

void ImageDisplayCustom::setRenderPanel( RenderPanel* rp )
{
    render_panel_ = rp;
}

void ImageDisplayCustom::selectionProcessed( int x1, int y1, int x2, int y2 )
{
//    std::cout << "Select Window: " << x1 << ", " << y1 << " -> " << x2 << ", " << y2 << std::endl;
//    //std::cout << "   full image rect: " << rect_dim_x1_ << ", " << rect_dim_y1_ << " -> " << rect_dim_x2_ << ", " << rect_dim_y2_ << std::endl;
//    //std::cout << "   window dimensions: " << render_panel_->width() << ", " << render_panel_->height() << std::endl;

//    // make sure selection is within image rect coordinates
//    float minx = std::min(x1, x2);
//    float _x1 = (minx < rect_dim_x1_ ? rect_dim_x1_ : (minx > rect_dim_x2_ ? rect_dim_x2_ : minx));
//    float maxx = std::max(x1, x2);
//    float _x2 = (maxx < rect_dim_x1_ ? rect_dim_x1_ : (maxx > rect_dim_x2_ ? rect_dim_x2_ : maxx));
//    float miny = std::min(y1, y2);
//    float _y1 = (miny < rect_dim_y1_ ? rect_dim_y1_ : (miny > rect_dim_y2_ ? rect_dim_y2_ : miny));
//    float maxy = std::max(y1, y2);
//    float _y2 = (maxy < rect_dim_y1_ ? rect_dim_y1_ : (maxy > rect_dim_y2_ ? rect_dim_y2_ : maxy));
//    //std::cout << "   corrected window: " << _x1 << ", " << _y1 << " -> " << _x2 << ", " << _y2 << std::endl;

//    // calculate image selection box dimensions -> texture coordinates
//    crop_x_offset_ = (_x1-rect_dim_x1_) * full_image_width_ / (rect_dim_x2_-rect_dim_x1_);
//    crop_y_offset_ = (_y1-rect_dim_y1_) * full_image_height_ / (rect_dim_y2_-rect_dim_y1_);
//    // and size
//    crop_width_  = ((_x2-_x1) * full_image_width_) / (rect_dim_x2_-rect_dim_x1_);
//    crop_height_ = ((_y2-_y1) * full_image_height_) / (rect_dim_y2_-rect_dim_y1_);

//    // create image request message
//    publishCropImageRequest();

//    std::cout << "   image selection rect: " << crop_x_offset_ << ", " << crop_y_offset_ << " -> " << (crop_x_offset_+crop_width_) << ", " << (crop_y_offset_+crop_height_) << std::endl;
}

void ImageDisplayCustom::changeFullImageResolution( int t )
{
//    std::cout << "Full image resolution changed:" << t << std::endl;
//    switch( t )
//    {
//    case IMAGE_RESOLUTION_FULL:
//    {
//        full_image_binning_ = 1;
//    }
//        break;
//    case IMAGE_RESOLUTION_2:
//    {
//        full_image_binning_ = 2;
//    }
//        break;
//    case IMAGE_RESOLUTION_4:
//    {
//        full_image_binning_ = 4;
//    }
//        break;
//    case IMAGE_RESOLUTION_8:
//    {
//        full_image_binning_ = 8;
//    }
//        break;
//    case IMAGE_RESOLUTION_16:
//    {
//        full_image_binning_ = 16;
//    }
//        break;
//    }

//    publishFullImageRequest();
}

void ImageDisplayCustom::changeCropImageResolution( int t )
{
//    std::cout << "Crop image resolution changed:" << t << std::endl;
//    switch( t )
//    {
//    case IMAGE_RESOLUTION_FULL:
//    {
//        crop_binning_ = 1;
//    }
//        break;
//    case IMAGE_RESOLUTION_2:
//    {
//        crop_binning_ = 2;
//    }
//        break;
//    case IMAGE_RESOLUTION_4:
//    {
//        crop_binning_ = 4;
//    }
//        break;
//    case IMAGE_RESOLUTION_8:
//    {
//        crop_binning_ = 8;
//    }
//        break;
//    case IMAGE_RESOLUTION_16:
//    {
//        crop_binning_ = 16;
//    }
//        break;
//    }

//    publishCropImageRequest();
}

void ImageDisplayCustom::changeCameraSpeed( int t )
{
//    std::cout << "Camera speed changed:" << t << std::endl;//(15.0f/(float)pow(3,t)) << std::endl;

//    if(t != 3)
//        publish_frequency_ = t;//15.0f/(float)pow(3,t); // 15 or whatever the max fps is
//    else
//        publish_frequency_ = 0.0f;

//    publishFullImageRequest();
//    publishCropImageRequest();
}


void ImageDisplayCustom::publishCropImageRequest()
{
    vigir_perception_msgs::DownSampledImageRequest cmd;

    cmd.binning_x = crop_binning_;
    cmd.binning_y = crop_binning_;
    cmd.roi.width = crop_width_;
    cmd.roi.height = crop_height_;
    cmd.roi.x_offset = crop_x_offset_;
    cmd.roi.y_offset = crop_y_offset_;
    if(publish_frequency_ == 0.0f)
        cmd.mode = 0;
    else
        cmd.mode = 1;
    cmd.publish_frequency = publish_frequency_;

    // publish image request for cropped image
    img_req_pub_crop_.publish( cmd );
}

void ImageDisplayCustom::publishFullImageRequest()
{
    vigir_perception_msgs::DownSampledImageRequest cmd;

    cmd.binning_x = full_image_binning_;
    cmd.binning_y = full_image_binning_;
    cmd.roi.width = full_image_width_;
    cmd.roi.height = full_image_height_;
    cmd.roi.x_offset = 0;
    cmd.roi.y_offset = 0;
    if(publish_frequency_ == 0.0f)
        cmd.mode = 0;
    else
        cmd.mode = 1;
    cmd.publish_frequency = publish_frequency_;

    // publish image request for full image
    img_req_pub_full_.publish( cmd );
}

} // namespace rviz

