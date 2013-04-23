/* 
 * ImageSelectionToolCustom class implementation.
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

#include "image_selection_tool_custom.h"
int theX1 = 0;
int theX2 = 0;
int theY1 = 0;
int theY2 = 0;
Ogre::Viewport* port =NULL;
namespace rviz
{

ImageSelectionToolCustom::ImageSelectionToolCustom()
  : Tool()
  , move_tool_( new MoveTool() )
  , selecting_( false )
  , sel_start_x_( 0 )
  , sel_start_y_( 0 )
  , moving_( false )
{
  shortcut_key_ = 's';
}

ImageSelectionToolCustom::~ImageSelectionToolCustom()
{
  delete move_tool_;
}

void ImageSelectionToolCustom::onInitialize()
{
  move_tool_->initialize( context_ );
}

void ImageSelectionToolCustom::activate()
{
  setStatus( "Click and drag to select objects on the screen." );
  context_->getSelectionManager()->setTextureSize(512);
  selecting_ = false;
  moving_ = false;
//  context_->getSelectionManager()->enableInteraction(true);
}

void ImageSelectionToolCustom::deactivate()
{
  //context_->getSelectionManager()->removeHighlight();
}

void ImageSelectionToolCustom::update(float wall_dt, float ros_dt)
{
  SelectionManager* sel_manager = context_->getSelectionManager();
 

  if(port)
  {
     sel_manager->highlight( port, theX1, theY1, theX2, theY2 );
  }
  /**if (!selecting_)
  {
    sel_manager->removeHighlight();
  }**/
}

int ImageSelectionToolCustom::processMouseEvent( ViewportMouseEvent& event )
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
    /**std::cout<<"tool selected x1 " <<sel_start_x_<<std::endl;
    std::cout<<"tool selected y1 " <<sel_start_y_<<std::endl;
    std::cout<<"tool selected x2 " <<event.x<<std::endl;
    std::cout<<"selected y2 " <<event.y<<std::endl;**/
    theX1 = sel_start_x_;
    theX2 = event.x;
    theY1 = sel_start_y_;
    theY2 = event.y;
    port = event.viewport;
    sel_manager->highlight( event.viewport, sel_start_x_, sel_start_y_, event.x, event.y );


    if( event.leftUp() )
    {
      SelectionManager::SelectType type = SelectionManager::Replace;

      M_Picked selection;

      /*if( event.shift() )
      {
        type = SelectionManager::Add;
      }
      else if( event.control() )
      {
        type = SelectionManager::Remove;
      }*/

      sel_manager->select( event.viewport, sel_start_x_, sel_start_y_, event.x, event.y, type );
      
      Q_EMIT select( sel_start_x_, sel_start_y_, event.x, event.y );

      selecting_ = false;
    }

    flags |= Render;
  }
  else if( moving_ )
  {
    //sel_manager->removeHighlight();

    flags = move_tool_->processMouseEvent( event );

    if( event.type == QEvent::MouseButtonRelease )
    {
      moving_ = false;
    }
  }
  else
  {
    sel_manager->highlight( event.viewport, theX1, theY1, theX2, theY2 );
  }

  return flags;
}

int ImageSelectionToolCustom::processKeyEvent( QKeyEvent* event, RenderPanel* panel )
{
  SelectionManager* sel_manager = context_->getSelectionManager();

  if( event->key() == Qt::Key_F )
  {
    sel_manager->focusOnSelection();
  }

  return Render;
}

void ImageSelectionToolCustom::unHighlight()
{
   context_->getSelectionManager()->removeHighlight();	
   port = NULL;
   theX1 = 0;
   theX2 = 0;
   theY1 = 0;
   theY2 = 0;
}

} // end namespace rviz

//#include <pluginlib/class_list_macros.h>
//PLUGINLIB_EXPORT_CLASS( rviz::ImageSelectionToolCustom, rviz::Tool )
