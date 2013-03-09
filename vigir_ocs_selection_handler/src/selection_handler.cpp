#include "selection_handler.h"

namespace vigir_ocs
{

SelectionHandler::SelectionHandler( QObject* parent ) 
	: QObject( parent )
{
}

SelectionHandler::~SelectionHandler()
{
}

void SelectionHandler::mousePressEvent( QMouseEvent* event ) 
{ 
	if( event->modifiers() == Qt::ControlModifier )
	{
		Q_EMIT select( event->x(), event->y() );
	}
}

} // namespace rviz
