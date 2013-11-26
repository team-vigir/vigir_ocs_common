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

#ifndef RVIZ_IMAGE_SELECTION_TOOL_H
#define RVIZ_IMAGE_SELECTION_TOOL_H

#include <QObject>

#include "rviz/tool.h"
#include "rviz/selection/forwards.h"

#include <vector>

namespace Ogre
{
class Viewport;
class Rectangle2D;
}

namespace rviz
{
class MoveTool;

class ImageSelectionToolCustom : public Tool
{
    Q_OBJECT
public:
    ImageSelectionToolCustom();
    virtual ~ImageSelectionToolCustom();

    virtual void onInitialize();

    virtual void activate();
    virtual void deactivate();

    virtual int processMouseEvent( ViewportMouseEvent& event );
    virtual int processKeyEvent( QKeyEvent* event, RenderPanel* panel );

    virtual void update(float wall_dt, float ros_dt);

    void setVisibilityBits(uint32_t vis_bit);
    bool isHighlightEnabled() { return highlight_enabled_; }

Q_SIGNALS:
    void select( int, int, int, int );
    void mouseHasMoved(int,int);

public Q_SLOTS:

    void unHighlight();

private:

    // control the highlight box being displayed while selecting
    void highlight(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2);
    void removeHighlight();

    void setHighlightRect(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2);

    MoveTool* move_tool_;

    bool selecting_;
    int sel_start_x_;
    int sel_start_y_;

    bool moving_;

    bool highlight_enabled_;

    struct Highlight
    {
        int x1;
        int y1;
        int x2;
        int y2;
        Ogre::Viewport* viewport;
    };
    Highlight highlight_;

    Ogre::Rectangle2D* highlight_rectangle_;
    Ogre::SceneNode* highlight_node_;
    int theX1,theX2,theY1,theY2;
    Ogre::Viewport* port;

    uint32_t vis_bit_;
};

}

#endif

