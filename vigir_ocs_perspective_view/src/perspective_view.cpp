/*
 * PerspectiveView class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on librviz_tutorials.
 *
 * Latest changes (12/08/2012):
 * - added support for joint manipulation?
 */

#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include "perspective_view.h"

namespace vigir_ocs
{
// Constructor for PerspectiveView.  This does most of the work of the class.
PerspectiveView::PerspectiveView( QWidget* parent, rviz::VisualizationManager* context )
    : Base3DView( context, "/world", parent )
{
}

// Destructor.
PerspectiveView::~PerspectiveView()
{

}
}
