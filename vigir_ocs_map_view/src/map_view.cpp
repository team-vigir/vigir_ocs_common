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

#include <flor_perception_msgs/EnvironmentRegionRequest.h>

#include "map_view.h"

namespace vigir_ocs
{
// Constructor for MapView.  This does most of the work of the class.
MapView::MapView( QWidget* parent )
    : Base3DView( "/world", parent )
{
    // block sending left/right mouse events to rviz by default
    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);

    // set the camera to be topdownortho
    rviz::ViewManager* view_man_ = manager_->getViewManager();
    view_man_->setCurrentFrom( view_man_->create( "rviz/TopDownOrtho" ) );

    // Add support for selection
    selection_tool_ = manager_->getToolManager()->addTool( "rviz/ImageSelectionToolCustom" );
    QObject::connect(mouse_event_handler_, SIGNAL(mouseLeftButton(bool,int,int)), this, SLOT(enableSelectionTool(bool,int,int)));
    QObject::connect(this, SIGNAL(unHighlight()), selection_tool_, SLOT(unHighlight()));

    Q_EMIT unHighlight();

    // create publisher for grid map
    grid_map_request_pub_ = n_.advertise<flor_perception_msgs::EnvironmentRegionRequest>( "/flor/worldmodel/ocs/gridmap_request", 1, false );

    // connect to selection display to query position/raycast
    QObject::connect(this, SIGNAL(queryPosition(int,int,Ogre::Vector3&)), selection_3d_display_, SLOT(queryPosition(int,int,Ogre::Vector3&)));
}

// Destructor.
MapView::~MapView()
{

}

void MapView::enableSelectionTool(bool activate, int x, int y)
{
    std::cout << "selection tool: " << activate << std::endl;
    if(activate)
    {
        selected_area_[0] = x;
        selected_area_[1] = y;
        // change to the selection tool and unblock events
        manager_->getToolManager()->setCurrentTool( selection_tool_ );
        ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::RightButton);
        ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::RightButton);
        ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::RightButton);
    }
    else
    {
        selected_area_[2] = x;
        selected_area_[3] = y;
        // block again and change back
        ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
        ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
        ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
        manager_->getToolManager()->setCurrentTool( move_camera_tool_ );

    }
}

void MapView::requestMap()
{
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    Ogre::Vector3 min, max;
    Q_EMIT queryPosition(selected_area_[0],selected_area_[1],min);
    Q_EMIT queryPosition(selected_area_[2],selected_area_[3],max);

    flor_perception_msgs::EnvironmentRegionRequest cmd;

    cmd.bounding_box_min.x = min.x;
    cmd.bounding_box_min.y = min.y;
    cmd.bounding_box_min.z = 0.1f;

    cmd.bounding_box_max.x = max.x;
    cmd.bounding_box_max.y = max.y;
    cmd.bounding_box_max.z = 2.0f;

    cmd.resolution = 0;

    grid_map_request_pub_.publish(cmd);

    Q_EMIT unHighlight();
}
}

