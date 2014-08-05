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
#include "rviz/default_plugin/view_controllers/fps_view_controller.h"
#include "rviz/views_panel.h"
#include "rviz/properties/vector_property.h"
#include <render_panel_custom.h>

#include "perspective_view.h"

namespace vigir_ocs
{
// Constructor for PerspectiveView.  This does most of the work of the class.
PerspectiveView::PerspectiveView( QWidget *parent, Base3DView* copy_from, std::string base_frame, std::string widget_name )
    : Base3DView( copy_from, base_frame, widget_name, parent )
{
    init();
}

PerspectiveView::PerspectiveView( Base3DView* copy_from, std::string base_frame, std::string widget_name, QWidget *parent )
    : Base3DView( copy_from, base_frame, widget_name, parent )
{
    init();
}

// Destructor.
PerspectiveView::~PerspectiveView()
{

}

void PerspectiveView::init()
{
    //subscribe to process camera transform
    camera_transform_sub_ = nh_.subscribe<geometry_msgs::Pose>( "/flor/ocs/set_camera_transform", 5, &PerspectiveView::processCameraTransform, this );

    view_change_timer_.start();
}

void PerspectiveView::processCameraTransform(const geometry_msgs::Pose::ConstPtr& msg)
{
    if(view_change_timer_.elapsed() > 100 && msg->orientation.x == 0 && msg->orientation.y == 0 && msg->orientation.z == 0
            && msg->position.x == 0 && msg->position.y == 0 && msg->position.z == 0)
    {
        if(manager_->getViewManager()->getCurrent()->getClassId() == "rviz/FPS")
        {
            view_change_timer_.start();

            manager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
        }
    }
    else if(view_change_timer_.elapsed() > 100)
    {
        //set the fps controller visible
        if(manager_->getViewManager()->getCurrent()->getClassId() == "rviz/Orbit")
        {
            view_change_timer_.start();

            manager_->getViewManager()->setCurrentViewControllerType("rviz/FPS");
        }
        else
        {
            rviz::FPSViewController* fps_view_controller_ = (rviz::FPSViewController*)(manager_->getViewManager()->getCurrent());
            fps_view_controller_->move(msg->position.x, msg->position.y, msg->position.z);
            fps_view_controller_->pitch(msg->orientation.x);
            fps_view_controller_->yaw(msg->orientation.z);
        }
    }
}

void PerspectiveView::timerEvent(QTimerEvent *event)
{
    // call the base3dview version of the timerevent
    Base3DView::timerEvent(event);
}

rviz::ViewController* PerspectiveView::getCurrentViewController()
{
    return manager_->getViewManager()->getCurrent();
}
}
