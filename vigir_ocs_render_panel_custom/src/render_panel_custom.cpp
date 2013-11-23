#include <QApplication>
#include <QMouseEvent>

#include "render_panel_custom.h"

namespace rviz
{

RenderPanelCustom::RenderPanelCustom( QWidget* parent )
  : RenderPanel( parent )
{
}

RenderPanelCustom::~RenderPanelCustom()
{
}

void RenderPanelCustom::setEventFilters(int function, bool block, Qt::KeyboardModifiers block_key_mod, Qt::MouseButtons block_mouse_buttons)
{
    BlockConfig config;
    config.block = block;
    config.block_key_modifiers = block_key_mod;
    config.block_mouse_buttons = block_mouse_buttons;
    block_config_[function] = config;
}

void RenderPanelCustom::mouseMoveEvent( QMouseEvent* event ) 
{
    Q_EMIT signalMouseMoveEvent( event );
    if(block_config_.find(MOUSE_MOVE_EVENT) != block_config_.end())
    {
        if(!block_config_[MOUSE_MOVE_EVENT].block &&
           !(event->modifiers() & block_config_[MOUSE_MOVE_EVENT].block_key_modifiers) &&
           !(event->buttons() & block_config_[MOUSE_MOVE_EVENT].block_mouse_buttons))
        {
            RenderPanel::mouseMoveEvent( event );
        }
    }
    else
        RenderPanel::mouseMoveEvent( event );
}

void RenderPanelCustom::mousePressEvent( QMouseEvent* event ) 
{
    Q_EMIT signalMousePressEvent( event );
    //if( !(event->buttons() & Qt::RightButton) ) // ignore right button events
    if(block_config_.find(MOUSE_PRESS_EVENT) != block_config_.end())
    {
        if(!block_config_[MOUSE_PRESS_EVENT].block &&
           !(event->modifiers() & block_config_[MOUSE_PRESS_EVENT].block_key_modifiers) &&
           !(event->buttons() & block_config_[MOUSE_PRESS_EVENT].block_mouse_buttons))
        {
            RenderPanel::mousePressEvent( event );
        }
    }
    else
        RenderPanel::mousePressEvent( event );
}

void RenderPanelCustom::mouseReleaseEvent( QMouseEvent* event ) 
{
    Q_EMIT signalMouseReleaseEvent( event );
    //if( !(event->buttons() & Qt::RightButton) ) // ignore right button events
    if(block_config_.find(MOUSE_RELEASE_EVENT) != block_config_.end())
    {
        if(!block_config_[MOUSE_RELEASE_EVENT].block &&
           !(event->modifiers() & block_config_[MOUSE_RELEASE_EVENT].block_key_modifiers) &&
           !(event->buttons() & block_config_[MOUSE_RELEASE_EVENT].block_mouse_buttons))
        {
            RenderPanel::mouseReleaseEvent( event );
        }
    }
    else
        RenderPanel::mouseReleaseEvent( event );
}

void RenderPanelCustom::mouseDoubleClickEvent( QMouseEvent* event ) 
{
    Q_EMIT signalMouseDoubleClickEvent( event );
    //if( !(event->buttons() & Qt::RightButton) ) // ignore right button events
    if(block_config_.find(MOUSE_DOUBLE_CLICK_EVENT) != block_config_.end())
    {
        if(!block_config_[MOUSE_DOUBLE_CLICK_EVENT].block &&
           !(event->modifiers() & block_config_[MOUSE_DOUBLE_CLICK_EVENT].block_key_modifiers) &&
           !(event->buttons() & block_config_[MOUSE_DOUBLE_CLICK_EVENT].block_mouse_buttons))
        {
            RenderPanel::mouseDoubleClickEvent( event );
        }
    }
    else
        RenderPanel::mouseDoubleClickEvent( event );
}


void RenderPanelCustom::leaveEvent ( QEvent * event )
{
    Q_EMIT signalMouseLeaveEvent( event );
    if(block_config_.find(MOUSE_LEAVE_EVENT) != block_config_.end())
    {
        if(!block_config_[MOUSE_LEAVE_EVENT].block/* &&
           !(event->modifiers() & block_config_[MOUSE_LEAVE_EVENT].block_key_modifiers) &&
           !(event->buttons() & block_config_[MOUSE_LEAVE_EVENT].block_mouse_buttons)*/)
        {
            RenderPanel::leaveEvent( event );
        }
    }
    else
        RenderPanel::leaveEvent( event );
}

void RenderPanelCustom::enterEvent( QEvent * event )
{
    Q_EMIT signalMouseEnterEvent( event );
}

void RenderPanelCustom::wheelEvent( QWheelEvent* event )
{
    Q_EMIT signalMouseWheelEvent( event );
    if(block_config_.find(MOUSE_WHEEL_EVENT) != block_config_.end())
    {
        if(!block_config_[MOUSE_WHEEL_EVENT].block &&
           !(event->modifiers() & block_config_[MOUSE_WHEEL_EVENT].block_key_modifiers) &&
           !(event->buttons() & block_config_[MOUSE_WHEEL_EVENT].block_mouse_buttons))
        {
            RenderPanel::wheelEvent( event );
        }
    }
    else
        RenderPanel::wheelEvent( event );
}

void RenderPanelCustom::keyPressEvent( QKeyEvent* event )
{
    if(block_config_.find(KEY_PRESS_EVENT) != block_config_.end())
    {
        if(!block_config_[KEY_PRESS_EVENT].block &&
           !(event->modifiers() & block_config_[KEY_PRESS_EVENT].block_key_modifiers)/* &&
           !(event->buttons() & block_config_[KEY_PRESS_EVENT].block_mouse_buttons)*/)
        {
            RenderPanel::keyPressEvent( event );
        }
    }
    else
        RenderPanel::keyPressEvent( event );
    Q_EMIT signalKeyPressEvent( event );
}

} // namespace rviz
