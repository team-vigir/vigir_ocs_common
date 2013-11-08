/* 
 * CameraDisplayCustom class definition.
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

#ifndef RVIZ_camera_display_custom_H
#define RVIZ_camera_display_custom_H

#include <QObject>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreRenderQueueListener.h>
#include "rviz/image/image_display_base.h"
#include "rviz/image/ros_image_texture.h"
#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"

#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <flor_perception_msgs/DownSampledImageRequest.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

class QMouseEvent;

namespace Ogre
{
class SceneNode;
class Rectangle2D;
class ManualObject;
class ViewportMouseEvent;
class Camera;
}

namespace rviz
{
class EnumProperty;
class FloatProperty;
class IntProperty;
class RenderPanel;
class RosTopicProperty;
class DisplayGroupVisibilityProperty;

class CameraDisplayCustom: public rviz::ImageDisplayBase,  public Ogre::RenderTargetListener, public Ogre::RenderQueueListener
{
    Q_OBJECT
public:
    CameraDisplayCustom();
    virtual ~CameraDisplayCustom();

    // Overrides from Display
    virtual void onInitialize();
    virtual void update( float wall_dt, float ros_dt );
    virtual void reset();

    // need to change these to be slots
    virtual void setRenderPanel( RenderPanel* rp );
    virtual void selectionProcessed( int x1, int y1, int x2, int y2 );

    // Overrides from Ogre::RenderTargetListener

    virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
    virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
    virtual void renderQueueStarted(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation);
    virtual void renderQueueEnded(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation);
    virtual void fixedFrameChanged();

    void setup();

    void setAlpha(float newAlpha);
    void updateSelectedAlpha(float newSelectedAlpha);
    void setZoom(float newZoom);
    void closeSelected();
    static const QString BACKGROUND;
    static const QString OVERLAY;
    static const QString BOTH;

public Q_SLOTS:
    void changeFullImageResolution( int );
    void changeCropImageResolution( int );
    void changeCameraSpeed( int );
    void changeCropCameraSpeed( int );

private Q_SLOTS:
    void forceRender();
    void updateAlpha();

    virtual void updateQueueSize();

    void updateCroppedTopic();
    void updateImgReqTopic();
    void updateImgReqCroppedTopic();

    void publishCropImageRequest();
    void publishFullImageRequest();

Q_SIGNALS:
    void updateFrameID( std::string );

protected:
    // overrides from Display
    virtual void onEnable();
    virtual void onDisable();

    // This is called by incomingMessage().
    virtual void processMessage(const sensor_msgs::Image::ConstPtr& msg);

    virtual void processCroppedImage(const sensor_msgs::Image::ConstPtr& msg);

    // The two functions that update our local variables for
    void processFullImageRequest(const flor_perception_msgs::DownSampledImageRequest::ConstPtr& msg);
    void processCropImageRequest(const flor_perception_msgs::DownSampledImageRequest::ConstPtr& msg);

private:
    enum
    {
        IMAGE_RESOLUTION_FULL = 0,
        IMAGE_RESOLUTION_2 = 1,
        IMAGE_RESOLUTION_4 = 2,
        IMAGE_RESOLUTION_8 = 3,
        IMAGE_RESOLUTION_16 = 4
    } DECIMATE_OPTIONS;

    void subscribe();
    void unsubscribe();
    void clear();
    void updateStatus();
    bool updateCamera(bool update_image);
    void caminfoCallback( const sensor_msgs::CameraInfo::ConstPtr& msg );

    // variables that define the full image rendering surface
    // for the background
    Ogre::SceneNode* bg_scene_node_;
    Ogre::Rectangle2D* bg_screen_rect_;
    Ogre::MaterialPtr bg_material_;
    ROSImageTexture texture_;

    // variables that define the full image rendering surface
    // for the background
    Ogre::SceneNode* fg_scene_node_;
    Ogre::Rectangle2D* fg_screen_rect_;
    Ogre::MaterialPtr fg_material_;

    // variables that define the cropped image rendering surface
    Ogre::Rectangle2D* screen_rect_selection_;
    Ogre::MaterialPtr material_selection_;
    ROSImageTexture texture_selection_;


    Ogre::Rectangle2D* fg_screen_rect_selection_;
    Ogre::MaterialPtr fg_material_selection_;
    ROSImageTexture fg_texture_selection_;

    // variables that define the rectangle that highlights selection
    Ogre::Rectangle2D* screen_rect_highlight_mask_;
    Ogre::Rectangle2D* screen_rect_highlight_bg_;
    Ogre::MaterialPtr material_highlight_;

    // reference to the main window render panel
    RenderPanel* render_panel_;

    //This is the lidar_point_cloud_viewer display that will hopefully show up.
    Display* lidar_point_cloud_viewer_;

    // ros publishers and subscribers
    ros::NodeHandle n_;
    ros::Publisher img_req_pub_crop_;
    ros::Publisher img_req_pub_full_;
    ros::Subscriber cropped_image_;
    ros::Subscriber img_req_sub_crop_;
    ros::Subscriber img_req_sub_full_;

    //This deals with the camera info
    message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo_sub_;
    tf::MessageFilter<sensor_msgs::CameraInfo>* caminfo_tf_filter_;

    sensor_msgs::CameraInfo::ConstPtr current_caminfo_;
    boost::mutex caminfo_mutex_;

    bool new_caminfo_;
    FloatProperty* alpha_property_;
    FloatProperty* zoom_property_;
    EnumProperty* image_position_property_;
    DisplayGroupVisibilityProperty* visibility_property_;
    bool caminfo_ok_;

    bool force_render_;
    uint32_t vis_bit_;

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

    // fps for both full and cropped images
    float publish_frequency_;
    float crop_publish_frequency_;

    // define *window* dimensions of the full image rendering surface -> necessary to calculate selection
    int rect_dim_x1_;
    int rect_dim_x2_;
    int rect_dim_y1_;
    int rect_dim_y2_;

    VisualizationManager* manager_;

    RosTopicProperty* cropped_topic_property_;
    RosTopicProperty* img_req_full_topic_property_;
    RosTopicProperty* img_req_cropped_topic_property_;

    // changing it so we hold the last information received
    sensor_msgs::CameraInfo::ConstPtr last_info_;
    sensor_msgs::Image::ConstPtr last_image_;

};

} // namespace rviz

#endif
