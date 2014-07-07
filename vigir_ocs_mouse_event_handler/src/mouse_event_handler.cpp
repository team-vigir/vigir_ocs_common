#include "mouse_event_handler.h"

#include <iostream>

namespace vigir_ocs
{

MouseEventHandler::MouseEventHandler( QObject* parent ) 
    : QObject( parent )
{
}

MouseEventHandler::~MouseEventHandler()
{
}

void MouseEventHandler::mousePressEvent( QMouseEvent* event ) 
{ 
    if( event->button() == Qt::LeftButton )
    {
        Qt::KeyboardModifiers keyMod = event->modifiers();
        bool shift = keyMod.testFlag(Qt::ShiftModifier);
        bool ctrl = keyMod.testFlag(Qt::ControlModifier);

        if( shift ) // as long as shift is pressed
        {
            Q_EMIT mouseLeftButtonShift( true, event->x(), event->y() );
        }
        if( ctrl ) // if only ctrl is pressed
        {
            Q_EMIT mouseLeftButtonCtrl( true, event->x(), event->y() );
        }

        if(!ctrl && !shift)
        {
            Q_EMIT mouseLeftButton( true, event->x(), event->y() );
        }
    }
    else if( event->button() == Qt::RightButton )
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
        else if( event->modifiers() == Qt::NoModifier )
        {
            Q_EMIT mouseLeftButton( false, event->x(), event->y() );
        }
    }
}

void MouseEventHandler::mouseDoubleClick(QMouseEvent * event)
{
    //should only handle left double click
    if( event->button() == Qt::LeftButton )
    {
        Q_EMIT signalMouseLeftDoubleClick(event->x(),event->y());
    }
}

} // namespace rviz
