// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef JSK_RVIZ_PLUGIN_OVERLAY_TEXT_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_OVERLAY_TEXT_DISPLAY_H_

#include "flor_ocs_msgs/OCSOverlayText.h"
#include <rviz/display.h>
#include "overlay_utils.h"
#include <std_msgs/ColorRGBA.h>
#include <rviz/properties/ros_topic_property.h>
#include "rviz/display_context.h"

#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreMaterial.h>
#include "OGRE/OgreRenderSystem.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreWindowEventUtilities.h"
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreRenderQueueListener.h>
#include <QElapsedTimer>


namespace rviz
{
    class RenderPanel;
}

namespace jsk_rviz_plugin
{
class OverlayTextDisplay : public rviz::Display, public Ogre::RenderTargetListener, public Ogre::RenderQueueListener
{
    Q_OBJECT
public:
    OverlayTextDisplay();
    virtual ~OverlayTextDisplay();

    // ogre calls
    virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
    virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
    virtual void renderQueueStarted(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation);
    virtual void renderQueueEnded(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation);
protected:
    OverlayObject::Ptr overlay_;

    int texture_width_;
    int texture_height_;

    QElapsedTimer timer_;
    int viewport_width_;
    int viewport_height_;
    // std_msgs::ColorRGBA bg_color_;
    // std_msgs::ColorRGBA fg_color_;
    QColor bg_color_;
    QColor fg_color_;
    int text_size_;
    int line_width_;
    std::string text_;
    std::string font_;
    float up_time_;
    float fade_in_;
    float fade_out_;
    float fade_in_timer_;
    float up_timer_;
    float fade_out_timer_;
    int row_;
    int column_;
    std::vector<rviz::RenderPanel*> render_panel_list_;
    rviz::RenderPanel* render_panel_; // this is the active render panels
        
    ros::Subscriber sub_;

    void setPositionFromAlignment();
    virtual void onInitialize();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void onEnable();
    virtual void onDisable();
    virtual void update(float wall_dt, float ros_dt);
    bool require_update_texture_;
    rviz::RosTopicProperty* update_topic_property_;
protected Q_SLOTS:
    void updateTopic();

public Q_SLOTS:
    void setRenderPanel(rviz::RenderPanel*);


    
private:
    void processMessage(const flor_ocs_msgs::OCSOverlayText::ConstPtr& msg);
};
}

#endif
