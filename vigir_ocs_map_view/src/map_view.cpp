/*
 * MapView class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on librviz_tutorials.
 *
 * Latest changes (12/08/2012):
 */

#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/view_manager.h"
#include "map_view.h"

namespace vigir_ocs
{
// Constructor for MapView.  This does most of the work of the class.
MapView::MapView( QWidget* parent )
    : Base3DView( "/pelvis", parent )
{
    // set the camera to be topdownortho
    rviz::ViewManager* view_man_ = manager_->getViewManager();
    view_man_->setCurrentFrom( view_man_->create( "rviz/TopDownOrtho" ) );
}

// Destructor.
MapView::~MapView()
{

}
}

