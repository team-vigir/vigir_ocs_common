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
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <QWidget>
#include <QPainter>

namespace jsk_rviz_plugin
{
OverlayTextDisplay::OverlayTextDisplay() : Display(),
    texture_width_(0), texture_height_(0),
    text_size_(14),
    line_width_(2),
    text_(""), font_(""),
    bg_color_(0, 0, 0, 0),
    fg_color_(255, 255, 255, 255),
    require_update_texture_(false),
    overlay_text_(NULL)
{
    update_topic_property_ = new rviz::RosTopicProperty(
                "Topic", "",
                ros::message_traits::datatype<flor_ocs_msgs::OCSOverlayText>(),
                "flor_ocs_msgs::OCSOverlayText topic to subscribe to.",
                this, SLOT( updateTopic() ));
    overtake_properties_property_ = new rviz::BoolProperty(
                "Overtake Properties", false,
                "overtake properties specified by message such as left, top and font",
                this, SLOT(updateOvertakeProperties()));
    top_property_ = new rviz::IntProperty(
                "top", 0,
                "top position",
                this, SLOT(updateTop()));
    top_property_->setMin(0);
    left_property_ = new rviz::IntProperty(
                "left", 0,
                "left position",
                this, SLOT(updateLeft()));
    left_property_->setMin(0);
    width_property_ = new rviz::IntProperty(
                "width", 128,
                "width position",
                this, SLOT(updateWidth()));
    width_property_->setMin(0);
    height_property_ = new rviz::IntProperty(
                "height", 128,
                "height position",
                this, SLOT(updateHeight()));
    height_property_->setMin(0);
    text_size_property_ = new rviz::IntProperty(
                "text size", 12,
                "text size",
                this, SLOT(updateTextSize()));
    text_size_property_->setMin(0);
    line_width_property_ = new rviz::IntProperty(
                "line width", 2,
                "line width",
                this, SLOT(updateLineWidth()));
    line_width_property_->setMin(0);
    fg_color_property_ = new rviz::ColorProperty(
                "Foreground Color", QColor(25, 255, 240),
                "Foreground Color",
                this, SLOT(updateFGColor()));
    fg_alpha_property_ = new rviz::FloatProperty(
                "Foreground Alpha", 0.8, "Foreground Alpha",
                this, SLOT(updateFGAlpha()));
    fg_alpha_property_->setMin(0.0);
    fg_alpha_property_->setMax(1.0);
    bg_color_property_ = new rviz::ColorProperty(
                "Background Color", QColor(0, 0, 0),
                "Background Color",
                this, SLOT(updateBGColor()));
    bg_alpha_property_ = new rviz::FloatProperty(
                "Background Alpha", 0.8, "Background Alpha",
                this, SLOT(updateBGAlpha()));
    bg_alpha_property_->setMin(0.0);
    bg_alpha_property_->setMax(1.0);
    font_property_ = new rviz::StringProperty(
                "font", "DejaVu Sans Mono",
                "font", this,
                SLOT(updateFont()));
}

OverlayTextDisplay::~OverlayTextDisplay()
{
    onDisable();
    //delete overlay_;
    delete update_topic_property_;
    delete top_property_;
    delete left_property_;
    delete width_property_;
    delete height_property_;
    delete text_size_property_;
    delete fg_color_property_;
    delete fg_alpha_property_;
    delete bg_color_property_;
    delete bg_alpha_property_;
    delete font_property_;
}

void OverlayTextDisplay::onEnable()
{
    subscribe();
}

void OverlayTextDisplay::onDisable()
{
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
    updateOvertakeProperties();
    updateTop();
    updateLeft();
    updateWidth();
    updateHeight();
    updateTextSize();
    updateFGColor();
    updateFGAlpha();
    updateBGColor();
    updateBGAlpha();
    updateFont();
    updateLineWidth();
    require_update_texture_ = true;
    subscribe();
}

void OverlayTextDisplay::update(float wall_dt, float ros_dt)
{
    if (!isEnabled()) {
        return;
    }
    if (!overlay_text_) {
        return;
    }

    //set colors based on fade timers
    if(fade_in_timer_ > 0)
    {
        //increase alpha as timer decreases
        fade_in_timer_ -= wall_dt;
        float a = 1.0 - (fade_in_timer_ / fade_in_);
        //set a to 0 if negative
        a = a>=0 ? a : 0;
        fg_color_.setAlpha(a  *230.0); // want max at 90%
        bg_color_.setAlpha(a  *127.0); // want max at 50%        

        //update text with new color values
        overlay_text_->setTextColor(fg_color_.redF(),fg_color_.greenF(),fg_color_.blueF(),fg_color_.alphaF());
        overlay_text_->setPanelColor(bg_color_.red(),bg_color_.green(),bg_color_.blue(),bg_color_.alpha());
    }
    else if(up_timer_ > 0)
    {
        //constant alpha
        up_timer_ -= wall_dt;
        fg_color_.setAlpha(1.0 * 230.0);
        bg_color_.setAlpha(1.0* 127.0);        
        //update text with new color values
        overlay_text_->setTextColor(fg_color_.redF(),fg_color_.greenF(),fg_color_.blueF(),fg_color_.alphaF());
        overlay_text_->setPanelColor(bg_color_.red(),bg_color_.green(),bg_color_.blue(),bg_color_.alpha());
    }
    else if(fade_out_timer_ > 0)
    {
        fade_out_timer_ -= wall_dt;
        //decrease alpha
        float a = (fade_out_timer_ / fade_out_);
        a = a>=0 ? a : 0;
        fg_color_.setAlpha(a * 230.0);
        bg_color_.setAlpha(a * 127.0);

        //update text with new color values
        overlay_text_->setTextColor(fg_color_.redF(),fg_color_.greenF(),fg_color_.blueF(),fg_color_.alphaF());
        overlay_text_->setPanelColor(bg_color_.red(),bg_color_.green(),bg_color_.blue(),bg_color_.alpha());

    }

    viewport_width_ = (Real) (OverlayManager::getSingleton().getViewportWidth());
    viewport_height_ = (Real) (OverlayManager::getSingleton().getViewportHeight());
    setPositionFromAlignment();

    overlay_text_->setPos(left_, top_);
}

void OverlayTextDisplay::setPositionFromAlignment()
{
    switch(column_)
    {
    case flor_ocs_msgs::OCSOverlayText::LEFTCOLUMN:
        left_ = 5; //minimal space from edge
        break;
    case flor_ocs_msgs::OCSOverlayText::CENTERCOLUMN:
        left_ = (viewport_width_ /2);
        break;
    case flor_ocs_msgs::OCSOverlayText::RIGHTCOLUMN:
        left_ = viewport_width_ - texture_width_ -5;
        break;
    }

    switch(row_)
    {
    case flor_ocs_msgs::OCSOverlayText::TOPROW:
        top_ = 5;
        break;
    case flor_ocs_msgs::OCSOverlayText::CENTERROW:
        top_ = (viewport_height_/2);
        break;
    case flor_ocs_msgs::OCSOverlayText::BOTTOMROW:
        top_ = viewport_height_ - texture_height_ -5;
        break;
    }

}

void OverlayTextDisplay::processMessage(const flor_ocs_msgs::OCSOverlayText::ConstPtr& msg)
{
    if (!isEnabled())
    {
        return;
    }
    if (!overlay_text_)
    {
        overlay_text_ = new OgreText();
    }

    // store message for update method
    text_ = msg->text;

    if (!overtake_properties_)
    {
        //calculate overlay position based on msg alignment
        row_ = msg->row;
        column_ = msg->column;

        texture_width_ = msg->width;
        texture_height_ = msg->height;
        font_ = msg->font;
        bg_color_ = QColor(msg->bg_color.r * 255.0,
                           msg->bg_color.g * 255.0,
                           msg->bg_color.b * 255.0,
                           msg->bg_color.a * 255.0);
        fg_color_ = QColor(msg->fg_color.r * 255.0,
                           msg->fg_color.g * 255.0,
                           msg->fg_color.b * 255.0,
                           msg->fg_color.a * 255.0);
        overlay_text_->setTextColor(msg->fg_color.r, msg->fg_color.g, msg->fg_color.b, msg->fg_color.a);
        text_size_ = msg->text_size;
        line_width_ = msg->line_width;        
        setPositionFromAlignment();
        up_time_ = msg->upTime;
        fade_in_ = msg->fadeIn;
        fade_out_ = msg->fadeOut;
        //setup timers to start fades
        fade_in_timer_ = fade_in_;
        up_timer_ = up_time_;
        fade_out_timer_ = fade_out_;
    }

    // font
    /*if (text_size_ != 0)
    {
        overlay_text_->setFont(font);
    }*/
    if (text_.length() > 0)
        overlay_text_->setText(text_.c_str());
}

void OverlayTextDisplay::updateOvertakeProperties()
{
    if (!overtake_properties_ && overtake_properties_property_->getBool()) {
        // read all the parameters from properties
        updateTop();
        updateLeft();
        updateWidth();
        updateHeight();
        updateTextSize();
        updateFGColor();
        updateFGAlpha();
        updateBGColor();
        updateBGAlpha();
        updateFont();
        updateLineWidth();
    }
    overtake_properties_ = overtake_properties_property_->getBool();
}

void OverlayTextDisplay::updateTop()
{
    top_ = top_property_->getInt();
}

void OverlayTextDisplay::updateLeft()
{
    left_ = left_property_->getInt();
}

void OverlayTextDisplay::updateWidth()
{
    texture_width_ = width_property_->getInt();
}

void OverlayTextDisplay::updateHeight()
{
    texture_height_ = height_property_->getInt();
}

void OverlayTextDisplay::updateTextSize()
{
    text_size_ = text_size_property_->getInt();
}

void OverlayTextDisplay::updateBGColor()
{
    QColor c = bg_color_property_->getColor();
    bg_color_.setRed(c.red());
    bg_color_.setGreen(c.green());
    bg_color_.setBlue(c.blue());
}

void OverlayTextDisplay::updateBGAlpha()
{
    bg_color_.setAlpha(bg_alpha_property_->getFloat() * 255.0);
}

void OverlayTextDisplay::updateFGColor()
{
    QColor c = fg_color_property_->getColor();
    fg_color_.setRed(c.red());
    fg_color_.setGreen(c.green());
    fg_color_.setBlue(c.blue());
}

void OverlayTextDisplay::updateFGAlpha()
{
    fg_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
}

void OverlayTextDisplay::updateFont()
{
    font_ = font_property_->getStdString();
}

void OverlayTextDisplay::updateLineWidth()
{
    line_width_ = line_width_property_->getInt();
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::OverlayTextDisplay, rviz::Display )
