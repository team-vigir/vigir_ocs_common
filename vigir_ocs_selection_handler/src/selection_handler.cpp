#include "selection_handler.h"

namespace vigir_ocs
{

SelectionHandler::SelectionHandler( QObject* parent ) 
    : QObject( parent ),
      xo(0),
      yo(0)
{
}

SelectionHandler::~SelectionHandler()
{
}

void SelectionHandler::mousePressEvent( QMouseEvent* event ) 
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

} // namespace rviz
