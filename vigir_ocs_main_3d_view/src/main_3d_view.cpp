/*
 * Main3DView class implementation.
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

#include "main_3d_view.h"

namespace vigir_ocs
{
// Constructor for Main3DView.  This does most of the work of the class.
Main3DView::Main3DView( QWidget* parent )
    : Base3DView( "/pelvis", parent )
{

}

// Destructor.
Main3DView::~Main3DView()
{

}
}
