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

#include "rviz/visualization_manager.h"
#include "rviz/view_controller.h"
#include "rviz/tool_manager.h"
#include "rviz/view_manager.h"
#include "rviz/display.h"
#include "rviz/default_plugin/view_controllers/orbit_view_controller.h"
#include <render_panel_custom.h>

#include "perspective_view.h"

namespace vigir_ocs
{
// Constructor for PerspectiveView.  This does most of the work of the class.
PerspectiveView::PerspectiveView( QWidget* parent, rviz::VisualizationManager* context )
    : Base3DView( context, "/world", parent )
{
    // set the camera to be topdownortho
    //rviz::ViewManager* view_man_ = manager_->getViewManager();
    orbit_view_controller_ = new rviz::OrbitViewController();//view_man_->create( "rviz/OrthoViewControllerCustom" );
    //view_man_->setCurrentFrom( ortho_view_controller_ );
    //ortho_view_controller_->initialize(context);
    orbit_view_controller_->initialize(manager_);
    render_panel_->setViewController(orbit_view_controller_);
}

// Destructor.
PerspectiveView::~PerspectiveView()
{

}

void PerspectiveView::timerEvent(QTimerEvent *event)
{
    // call the base3dview version of the timerevent
    Base3DView::timerEvent(event);

    orbit_view_controller_->update(0,0);

//    float lastFPS, avgFPS, bestFPS, worstFPS;

//    render_panel_->getRenderWindow()->getStatistics( lastFPS, avgFPS, bestFPS, worstFPS );
//    std::cout << "Camera (" << /*ortho_view_controller_->subProp( "View Plane" )->getValue(). <<*/ "): " << lastFPS << ", " << avgFPS << ", " << bestFPS << ", " << worstFPS << std::endl;

    //std::cout << "camera position: " << render_panel_->getCamera()->getPosition().x << ", " << render_panel_->getCamera()->getPosition().z << ", " << render_panel_->getCamera()->getPosition().z << std::endl;
    //std::cout << "camera direction: " << render_panel_->getCamera()->getDirection().x << ", " << render_panel_->getCamera()->getDirection().z << ", " << render_panel_->getCamera()->getDirection().z << std::endl;
}

rviz::ViewController* PerspectiveView::getCurrentViewController()
{
    return orbit_view_controller_;
}
}
