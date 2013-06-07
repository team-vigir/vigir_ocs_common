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
#include "rviz/tool_manager.h"
#include "rviz/view_manager.h"
#include "rviz/display.h"
#include <render_panel_custom.h>
#include "map_view.h"

namespace vigir_ocs
{
// Constructor for MapView.  This does most of the work of the class.
MapView::MapView( QWidget* parent )
    : Base3DView( "/world", parent )
{
    // set the camera to be topdownortho
    rviz::ViewManager* view_man_ = manager_->getViewManager();
    view_man_->setCurrentFrom( view_man_->create( "rviz/TopDownOrtho" ) );

    //rviz::TopDownOrthoCustom* camera_controller = new rviz::TopDownOrthoCustom();
    //camera_controller->initialize( render_panel_->getManager() );
    //render_panel_->setViewController( camera_controller );
    //view_man_->setCurrentFrom( camera_controller );

    // Add support for selection
    //selection_tool_ = manager_->getToolManager()->addTool( "rviz/ImageSelectionToolCustom" );

    //manager_->getToolManager()->setCurrentTool( selection_tool_ );
}

// Destructor.
MapView::~MapView()
{

}
}

