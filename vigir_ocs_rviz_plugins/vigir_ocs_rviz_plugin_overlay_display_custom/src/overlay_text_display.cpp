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

#include "overlay_text_display.h"
#include <OGRE/OgreMaterialManager.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/render_panel.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OgreRenderWindow.h>
#include <QPainter>

namespace jsk_rviz_plugin
{
OverlayTextDisplay::OverlayTextDisplay() : Display(),
    texture_width_(0), texture_height_(0),
    text_size_(14),
    line_width_(2),
    text_(""), font_(""),
    bg_color_(0, 0, 0, 0),
    fg_color_(255, 255, 255, 255.0),
    row_(-1),
    column_(-1),
    require_update_texture_(false)
{
    update_topic_property_ = new rviz::RosTopicProperty(
                "Topic", "",
                ros::message_traits::datatype<flor_ocs_msgs::OCSOverlayText>(),
                "flor_ocs_msgs::OCSOverlayText topic to subscribe to.",
                this, SLOT( updateTopic() ));
}


OverlayTextDisplay::~OverlayTextDisplay()
{
    onDisable();
    //delete overlay_;
    delete update_topic_property_;
}

void OverlayTextDisplay::onEnable()
{
    if (overlay_) {
        overlay_->show();
    }
    subscribe();
}

void OverlayTextDisplay::onDisable()
{
    if (overlay_) {
        overlay_->hide();
    }
    unsubscribe();
}

void OverlayTextDisplay::unsubscribe()
{
    sub_.shutdown();
}

void OverlayTextDisplay::subscribe()
{
    std::string topic_name = update_topic_property_->getTopicStd();
    if (topic_name.length() > 0 && topic_name != "/") {
        sub_ = ros::NodeHandle().subscribe(topic_name, 1, &OverlayTextDisplay::processMessage, this);
    }
}

void OverlayTextDisplay::updateTopic()
{
    unsubscribe();
    subscribe();
}

// only the first time
void OverlayTextDisplay::onInitialize()
{
    onEnable();
    updateTopic();
    require_update_texture_ = true;
    context_->getSceneManager()->addRenderQueueListener(this);
}

void OverlayTextDisplay::setRenderPanel( rviz::RenderPanel* rp )
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

void OverlayTextDisplay::preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt )
{
    if(overlay_)
    {
        int view_id = 0;

        for(int i = 0; i < render_panel_list_.size(); i++)
        {
            if(render_panel_list_[i]->getRenderWindow() == (Ogre::RenderWindow*)evt.source)
                view_id = i;
        }
       // ROS_ERROR("num viewports %d",render_panel_list_[view_id]->getRenderWindow()->getNumViewports());
        //grap viewport of primary view
        Ogre::Viewport * vp = this->render_panel_list_[view_id]->getRenderWindow()->getViewport(0);
        viewport_width_ = vp->getActualWidth();
        viewport_height_ = vp->getActualHeight();
       // ROS_ERROR("W %d H: %d", viewport_width_,viewport_height_);

        //if a msg has ever been received
        if(row_ != -1)
            setPositionFromAlignment();
    }
}

void OverlayTextDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{

}

void OverlayTextDisplay::renderQueueStarted(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation)
{

}

void OverlayTextDisplay::renderQueueEnded(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& repeatThisInvocation)
{

}

void OverlayTextDisplay::update(float wall_dt, float ros_dt)
{
    if (!require_update_texture_) {
        return;
    }
    if (!isEnabled()) {
        return;
    }
    if (!overlay_) {
        return;
    }
    overlay_->updateTextureSize(texture_width_, texture_height_);
    {

        //set colors based on fade timers
        if(fade_in_timer_>0)
        {
            //increase alpha as timer decreases
            fade_in_timer_ -= wall_dt;
            float a = 1.0 - (fade_in_timer_ / fade_in_);
            //set a to 0 if negative
            a = a>=0 ? a : 0;
            //fg_color_.setAlpha(a  *230.0); // want max at 90%
            //bg_color_.setAlpha(a  *10.0); // want max at 50%
        }
        else if(up_timer_ >0)
        {
            up_timer_ -= wall_dt;
            //fg_color_.setAlpha(1.0 * 230.0);
            ///bg_color_.setAlpha(1.0* 10.0);
        }
        else if(fade_out_timer_ > 0)
        {
            fade_out_timer_ -= wall_dt;
            float a = (fade_out_timer_ / fade_out_);
            a = a>=0 ? a : 0;
            //fg_color_.setAlpha(a * 230.0);
            //bg_color_.setAlpha(a * 10.0);
        }

        ScopedPixelBuffer buffer = overlay_->getBuffer();
        QImage Hud = buffer.getQImage(*overlay_, bg_color_);
        QPainter painter( &Hud );
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(fg_color_, line_width_ || 1, Qt::SolidLine));
        uint16_t w = overlay_->getTextureWidth();
        uint16_t h = overlay_->getTextureHeight();

        // font
        if (text_size_ != 0) {
            //QFont font = painter.font();
            QFont font(font_.length() > 0 ? font_.c_str(): "Arial");
            font.setPointSize(text_size_);
            font.setBold(true);
            painter.setFont(font);
        }
        if (text_.length() > 0) {
            //painter.drawText(0, 0, w, h, Qt::TextWordWrap | Qt::AlignLeft,
            painter.drawText(0, 0, w, h,
                             Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
                             text_.c_str());
        }
        painter.end();
    }
    overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
    //require_update_texture_ = false;
}

void OverlayTextDisplay::setPositionFromAlignment()
{
    int left;
    int top;
    switch(row_)
    {
    case flor_ocs_msgs::OCSOverlayText::LEFTCOLUMN:
        left = 0;
        break;
    case flor_ocs_msgs::OCSOverlayText::CENTERCOLUMN:
        left = viewport_width_ /2;
        break;
    case flor_ocs_msgs::OCSOverlayText::RIGHTCOLUMN:
        left = viewport_width_ - texture_width_;
        break;
    }

    switch(column_)
    {
    case flor_ocs_msgs::OCSOverlayText::TOPROW:
        top = 0;
        break;
    case flor_ocs_msgs::OCSOverlayText::CENTERROW:
        top = viewport_height_/2;
        break;
    case flor_ocs_msgs::OCSOverlayText::BOTTOMROW:
        top = viewport_height_ - texture_height_;
        break;
    }
    ROS_ERROR("Left: %d Top: %d based on: %d %d", left,top, viewport_width_,viewport_height_);
    overlay_->setPosition(left, top);

}

void OverlayTextDisplay::processMessage
(const flor_ocs_msgs::OCSOverlayText::ConstPtr& msg)
{
    if (!isEnabled()) {
        return;
    }
    if (!overlay_) {
        static int count = 0;
        rviz::UniformStringStream ss;
        ss << "OverlayTextDisplayObject" << count++;
        overlay_.reset(new OverlayObject(ss.str()));
        overlay_->show();
    }
    if (overlay_) {
        if (msg->action == flor_ocs_msgs::OCSOverlayText::DELETE) {
            overlay_->hide();
        }
        else if (msg->action == flor_ocs_msgs::OCSOverlayText::ADD) {
            overlay_->show();
        }
    }
    texture_width_ = msg->width;
    texture_height_ = msg->height;
    //calculate overlay position based on msg alignment
    if (overlay_)
    {
        row_ = msg->row;
        column_ = msg->column;

        //setPositionFromAlignment(msg->row,msg->column);
        //wanted to set position manually?
        if(!msg->row && !msg->column)
            overlay_->setPosition(msg->left, msg->top);
    }
    // store message for update method
    text_ = msg->text;
    font_ = msg->font;
    //alpha handled in update for animating
    bg_color_ = QColor(msg->bg_color.r * 255.0,
                       msg->bg_color.g * 255.0,
                       msg->bg_color.b * 255.0,
                       .2 * 255.0);
    fg_color_ = QColor(msg->fg_color.r * 255.0,
                       msg->fg_color.g * 255.0,
                       msg->fg_color.b * 255.0,
                       255.0);
    text_size_ = msg->text_size;
    line_width_ = msg->line_width;
    require_update_texture_ = true;
    up_time_ = msg->upTime;
    fade_in_ = msg->fadeIn;
    fade_out_ = msg->fadeOut;
    //setup timers to start fades
    fade_in_timer_ = fade_in_;
    up_timer_ = up_time_;
    fade_out_timer_ = fade_out_;

}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::OverlayTextDisplay, rviz::Display )
