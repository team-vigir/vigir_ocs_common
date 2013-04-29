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

void RenderPanelCustom::mouseMoveEvent( QMouseEvent* event ) 
{ 
	Q_EMIT signalMouseMoveEvent( event );
	RenderPanel::mouseMoveEvent( event );
}

void RenderPanelCustom::mousePressEvent( QMouseEvent* event ) 
{ 
	Q_EMIT signalMousePressEvent( event );
    if( !(event->buttons() & Qt::RightButton) ) // ignore right button events
        RenderPanel::mousePressEvent( event );
}

void RenderPanelCustom::mouseReleaseEvent( QMouseEvent* event ) 
{ 
	Q_EMIT signalMouseReleaseEvent( event );
    if( !(event->buttons() & Qt::RightButton) ) // ignore right button events
        RenderPanel::mouseReleaseEvent( event );
}

void RenderPanelCustom::mouseDoubleClickEvent( QMouseEvent* event ) 
{ 
	Q_EMIT signalMouseDoubleClickEvent( event );
    if( !(event->buttons() & Qt::RightButton) ) // ignore right button events
        RenderPanel::mouseDoubleClickEvent( event );
}


void RenderPanelCustom::leaveEvent ( QEvent * event )
{
	Q_EMIT signalMouseLeaveEvent( event );
	RenderPanel::leaveEvent( event ); 
}

void RenderPanelCustom::wheelEvent( QWheelEvent* event )
{
 	Q_EMIT signalMouseWheelEvent( event );
	RenderPanel::wheelEvent( event ); 
}

void RenderPanelCustom::keyPressEvent( QKeyEvent* event )
{
	Q_EMIT signalKeyPressEvent( event );
	RenderPanel::keyPressEvent( event ); 
}

} // namespace rviz
