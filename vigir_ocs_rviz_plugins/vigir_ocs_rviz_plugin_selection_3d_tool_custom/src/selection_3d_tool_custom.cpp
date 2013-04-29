/* 
 * Selection3DToolCustom class implementation.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/11/2012):
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

#include <QKeyEvent>

#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTextureManager.h>

#include <ros/time.h>

#include "move_tool.h"

#include "rviz/ogre_helpers/camera_base.h"
#include "rviz/ogre_helpers/qt_ogre_render_window.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"

#include "selection_3d_tool_custom.h"

namespace rviz
{

Selection3DToolCustom::Selection3DToolCustom()
  : Tool()
  , move_tool_( new MoveTool() )
  , selecting_( false )
  , sel_start_x_( 0 )
  , sel_start_y_( 0 )
  , moving_( false )
{
  shortcut_key_ = 's';
}

Selection3DToolCustom::~Selection3DToolCustom()
{
  delete move_tool_;
}

void Selection3DToolCustom::onInitialize()
{
  move_tool_->initialize( context_ );
}

void Selection3DToolCustom::activate()
{
  setStatus( "Click and drag to select objects on the screen." );
  selecting_ = false;
  moving_ = false;
}

void Selection3DToolCustom::deactivate()
{
}

void Selection3DToolCustom::update(float wall_dt, float ros_dt)
{

}

int Selection3DToolCustom::processMouseEvent( ViewportMouseEvent& event )
{
  SelectionManager* sel_manager = context_->getSelectionManager();

  int flags = 0;

  if( event.alt() )
  {
    moving_ = true;
    selecting_ = false;
  }
  else
  {
    moving_ = false;

    if( event.leftDown() )
    {
      selecting_ = true;

      sel_start_x_ = event.x;
      sel_start_y_ = event.y;
    }
  }

  if( selecting_ )
  {
    if( event.leftUp() )
    {
      Q_EMIT select( sel_start_x_, sel_start_y_, event.x, event.y );

      selecting_ = false;
    }

    flags |= Render;
  }
  else if( moving_ )
  {
    flags = move_tool_->processMouseEvent( event );

    if( event.type == QEvent::MouseButtonRelease )
    {
      moving_ = false;
    }
  }

  return flags;
}

int Selection3DToolCustom::processKeyEvent( QKeyEvent* event, RenderPanel* panel )
{
  SelectionManager* sel_manager = context_->getSelectionManager();

  if( event->key() == Qt::Key_F )
  {
    sel_manager->focusOnSelection();
  }

  return Render;
}

} // end namespace rviz

//#include <pluginlib/class_list_macros.h>
//PLUGINLIB_EXPORT_CLASS( rviz::Selection3DToolCustom, rviz::Tool )
