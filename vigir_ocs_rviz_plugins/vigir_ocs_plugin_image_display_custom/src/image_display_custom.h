/* 
 * ImageDisplayCustom class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/04/2012):
 *
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

#ifndef RVIZ_IMAGE_DISPLAY_CUSTOM_H
#define RVIZ_IMAGE_DISPLAY_CUSTOM_H

#include <QObject>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderTargetListener.h>

#include "rviz/image/image_display_base.h"
#include "rviz/image/ros_image_texture.h"
#include "rviz/render_panel.h"

#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/ros.h>

#include <image_transport/image_transport.h>

class QMouseEvent;

namespace Ogre
{
class SceneNode;
class Rectangle2D;
class ViewportMouseEvent;
}

namespace rviz
{

class ImageDisplayCustom: public rviz::ImageDisplayBase
{
Q_OBJECT
public:
  ImageDisplayCustom();
  virtual ~ImageDisplayCustom();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();

	// need to change these to be slots
  virtual void setRenderPanel( RenderPanel* rp );
  virtual void selectionProcessed( int x1, int y1, int x2, int y2 );

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  // This is called by incomingMessage(). 
  virtual void processMessage(const sensor_msgs::Image::ConstPtr& msg);
  virtual void processCroppedImage(const sensor_msgs::Image::ConstPtr& msg);

private:
  void clear();
  void updateStatus();
	
	// variables that define the full image rendering surface
  Ogre::Rectangle2D* screen_rect_;
  Ogre::MaterialPtr material_;
  ROSImageTexture texture_;
  
  // variables that define the cropped image rendering surface
  Ogre::Rectangle2D* screen_rect_selection_;
  Ogre::MaterialPtr material_selection_;
  ROSImageTexture texture_selection_;
  
  // variables that define the rectangle that highlights selection
  Ogre::Rectangle2D* screen_rect_highlight_;
  Ogre::MaterialPtr material_highlight_;

	// reference to the main window render panel
  RenderPanel* render_panel_;

	// ros publishers and subscribers
	ros::NodeHandle n_;
	ros::Publisher img_req_pub_crop_;
	ros::Publisher img_req_pub_full_;
	ros::Subscriber cropped_image_;

	// full image info	
	int full_image_width_;
	int full_image_height_;
	int full_image_binning_;
	
	// crop image info
	float crop_x_offset_;
	float crop_y_offset_;
	float crop_width_;
	float crop_height_;
    float crop_binning_;
	
	// define *window* dimensions of the full image rendering surface -> necessary to calculate selection
	int rect_dim_x1_;
	int rect_dim_x2_;
	int rect_dim_y1_;
	int rect_dim_y2_;
};

} // namespace rviz

#endif
