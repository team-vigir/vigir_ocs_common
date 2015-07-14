/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
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
@TODO_ADD_AUTHOR_INFO
#include <map_view_custom.h>

#include <pluginlib/class_list_macros.h>

#include <QCloseEvent>
#include <QMenuBar>

namespace rqt {

MapViewCustom::MapViewCustom()
  : rqt_gui_cpp::Plugin()
  , context_(0)
  , widget_(0)
{
  setObjectName("MapViewCustom");
}

MapViewCustom::~MapViewCustom()
{
}

void MapViewCustom::initPlugin(qt_gui_cpp::PluginContext& context)
{
  context_ = &context;

  widget_ = new MapViewWidget();
  context.addWidget(widget_);

  // trigger deleteLater for plugin when widget or frame is closed
  widget_->installEventFilter(this);
}

bool MapViewCustom::eventFilter(QObject* watched, QEvent* event)
{
  if (watched == widget_ && event->type() == QEvent::Close)
  {
    event->ignore();
    context_->closePlugin();
    return true;
  }

  return QObject::eventFilter(watched, event);
}

}

PLUGINLIB_EXPORT_CLASS(rqt::MapViewCustom, rqt_gui_cpp::Plugin)
