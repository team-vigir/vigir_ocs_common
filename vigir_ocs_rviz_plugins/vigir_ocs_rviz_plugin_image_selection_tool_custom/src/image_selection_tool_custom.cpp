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

namespace rviz
{

ImageSelectionToolCustom::ImageSelectionToolCustom()
    : Tool()
    , move_tool_( new MoveTool() )
    , selecting_( false )
    , sel_start_x_( 0 )
    , sel_start_y_( 0 )
    , moving_( false )
    , theX1(0)
    , theX2(0)
    , theY1(0)
    , theY2(0)
    , port(NULL)
{
    shortcut_key_ = 's';
}

ImageSelectionToolCustom::~ImageSelectionToolCustom()
{
    delete move_tool_;

    highlight_node_->getParentSceneNode()->removeAndDestroyChild(highlight_node_->getName());
    delete highlight_rectangle_;
}

void ImageSelectionToolCustom::onInitialize()
{
    move_tool_->initialize( context_ );

    // Create our highlight rectangle
    Ogre::SceneManager* scene_manager = context_->getSceneManager();
    highlight_node_ = scene_manager->getRootSceneNode()->createChildSceneNode();

    std::stringstream ss;
    static int count = 0;
    ss << "ImageSelectionRect" << count++;
    highlight_rectangle_ = new Ogre::Rectangle2D(true);

    const static uint32_t texture_data[1] = { 0xffff0070 };
    Ogre::DataStreamPtr pixel_stream;
    pixel_stream.bind(new Ogre::MemoryDataStream( (void*)&texture_data[0], 4 ));

    Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().loadRawData(ss.str() + "Texture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pixel_stream, 1, 1, Ogre::PF_R8G8B8A8, Ogre::TEX_TYPE_2D, 0);

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setLightingEnabled(false);
    //material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    highlight_rectangle_->setMaterial(material->getName());
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    highlight_rectangle_->setBoundingBox(aabInf);
    highlight_rectangle_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY + 4);
    material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material->setCullingMode(Ogre::CULL_NONE);

    Ogre::TextureUnitState* tex_unit = material->getTechnique(0)->getPass(0)->createTextureUnitState();
    tex_unit->setTextureName(tex->getName());
    tex_unit->setTextureFiltering( Ogre::TFO_NONE );

    highlight_node_->attachObject(highlight_rectangle_);
}

void ImageSelectionToolCustom::activate()
{
    setStatus( "Click and drag to select objects on the screen." );
    //context_->getSelectionManager()->setTextureSize(512);
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
    //SelectionManager* sel_manager = context_->getSelectionManager();


    //if(port)
    //{
    //   sel_manager->highlight( port, theX1, theY1, theX2, theY2 );
    //}


    highlight_node_->setVisible(highlight_enabled_);

    if (highlight_enabled_)
    {
        setHighlightRect(highlight_.viewport, highlight_.x1, highlight_.y1, highlight_.x2, highlight_.y2);
    }
}

int ImageSelectionToolCustom::processMouseEvent( ViewportMouseEvent& event )
{
    //SelectionManager* sel_manager = context_->getSelectionManager();
    Q_EMIT mouseHasMoved(event.x, event.y);
    int flags = 0;

    moving_ = false;

    if( event.leftDown() )
    {
        selecting_ = true;

        sel_start_x_ = event.x;
        sel_start_y_ = event.y;
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
        highlight( event.viewport, sel_start_x_, sel_start_y_, event.x, event.y );


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

            //sel_manager->select( event.viewport, sel_start_x_, sel_start_y_, event.x, event.y, type );
            removeHighlight();

            Q_EMIT select( sel_start_x_, sel_start_y_, event.x, event.y );

            selecting_ = false;
        }

        flags |= Render;
    }
    else
    {
        highlight( event.viewport, theX1, theY1, theX2, theY2 );
    }

    return flags;
}

int ImageSelectionToolCustom::processKeyEvent( QKeyEvent* event, RenderPanel* panel )
{
    //SelectionManager* sel_manager = context_->getSelectionManager();

    if( event->key() == Qt::Key_F )
    {
        //sel_manager->focusOnSelection();
    }

    return Render;
}

void ImageSelectionToolCustom::unHighlight()
{
    removeHighlight();
    port = NULL;
    theX1 = 0;
    theX2 = 0;
    theY1 = 0;
    theY2 = 0;
    highlight_node_->setVisible(false);
}

void ImageSelectionToolCustom::highlight(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2)
{
    if(viewport != NULL)
    {
        highlight_enabled_ = true;

        highlight_.viewport = viewport;
        highlight_.x1 = x1;
        highlight_.y1 = y1;
        highlight_.x2 = x2;
        highlight_.y2 = y2;
    }
}

void ImageSelectionToolCustom::setHighlightRect(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2)
{
    if(viewport != NULL)
    {
        float nx1 = ((float)x1 / viewport->getActualWidth()) * 2 - 1;
        float nx2 = ((float)x2 / viewport->getActualWidth()) * 2 - 1;
        float ny1 = -(((float)y1 / viewport->getActualHeight()) * 2 - 1);
        float ny2 = -(((float)y2 / viewport->getActualHeight()) * 2 - 1);

        nx1 = nx1 < -1 ? -1 : (nx1 > 1 ? 1 : nx1);
        ny1 = ny1 < -1 ? -1 : (ny1 > 1 ? 1 : ny1);
        nx2 = nx2 < -1 ? -1 : (nx2 > 1 ? 1 : nx2);
        ny2 = ny2 < -1 ? -1 : (ny2 > 1 ? 1 : ny2);

        highlight_rectangle_->setCorners(nx1, ny1, nx2, ny2);
    }
}

void ImageSelectionToolCustom::removeHighlight()
{
    highlight_enabled_ = false;
}

} // end namespace rviz

//#include <pluginlib/class_list_macros.h>
//PLUGINLIB_EXPORT_CLASS( rviz::ImageSelectionToolCustom, rviz::Tool )
