#include "mouse_event_handler.h"

namespace vigir_ocs
{

MouseEventHandler::MouseEventHandler( QObject* parent ) 
    : QObject( parent ),
      xo(0),
      yo(0)
{
}

MouseEventHandler::~MouseEventHandler()
{
}

void MouseEventHandler::mousePressEvent( QMouseEvent* event ) 
{ 
    if( event->buttons() & Qt::LeftButton )
    {
        if( event->modifiers() & Qt::ShiftModifier ) // as long as shift is pressed
        {
            Q_EMIT selectROI( event->x(), event->y() );
        }
        else if( event->modifiers() == Qt::ControlModifier ) // if only ctrl is pressed
        {
            Q_EMIT select( event->x(), event->y() );
            xo = event->x();
            yo = event->y();
        }
    }
    else if( event->buttons() & Qt::RightButton )
    {
		Q_EMIT createContextMenu( event->x(), event->y() );
    }
}

} // namespace rviz
