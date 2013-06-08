#include "mouse_event_handler.h"

#include <iostream>

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
            Q_EMIT mouseLeftButtonShift( true, event->x(), event->y() );
        }
        else if( event->modifiers() == Qt::ControlModifier ) // if only ctrl is pressed
        {
            Q_EMIT mouseLeftButtonCtrl( true, event->x(), event->y() );
            xo = event->x();
            yo = event->y();
        }
    }
    else if( event->buttons() & Qt::RightButton )
    {
        Q_EMIT mouseRightButton( true, event->x(), event->y() );
    }
}

void MouseEventHandler::mouseReleaseEvent( QMouseEvent* event )
{
    //std::cout << "mouse release event: " << event->button() << " " << event->modifiers() << std::endl;
    if( event->button() == Qt::LeftButton )
    {
        //std::cout << "left button" << std::endl;
        if( event->modifiers() & Qt::ShiftModifier ) // as long as shift is pressed
        {
            //std::cout << "shift" << std::endl;
            // need to emit selection signal here
            Q_EMIT mouseLeftButtonShift( false, event->x(), event->y() );
        }
    }
}

} // namespace rviz
