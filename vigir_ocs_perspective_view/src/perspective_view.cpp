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
    camera_transform_sub_ = nh_.subscribe( "/flor/ocs/set_camera_transform", 5, &PerspectiveView::processCameraTransform, this );

    view_change_timer_.start();
}

void PerspectiveView::processCameraTransform(const geometry_msgs::Pose::ConstPtr msg)
{
    if(render_panel_->getCamera()->getPosition().x != 0 && render_panel_->getCamera()->getPosition().y != 0 && render_panel_->getCamera()->getPosition().z != 0
            && render_panel_->getCamera()->getOrientation().x != 0 && render_panel_->getCamera()->getOrientation().y != 0 && render_panel_->getCamera()->getOrientation().z != 0 && render_panel_->getCamera()->getOrientation().w != 1
            && msg->orientation.x == 0 && msg->orientation.y == 0 && msg->orientation.z == 0
            && msg->position.x == 0 && msg->position.y == 0 && msg->position.z == 0)
    {
        if(manager_->getViewManager()->getCurrent()->getClassId() == "rviz/FPS")
        {
            manager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
        }
    }
    else
    {
        //set the fps controller visible
        if( render_panel_->getCamera()->getPosition().x != 0 && render_panel_->getCamera()->getPosition().y != 0 && render_panel_->getCamera()->getPosition().z != 0
                && render_panel_->getCamera()->getOrientation().x != 0 && render_panel_->getCamera()->getOrientation().y != 0 && render_panel_->getCamera()->getOrientation().z != 0 && render_panel_->getCamera()->getOrientation().w != 1
                && manager_->getViewManager()->getCurrent()->getClassId() == "rviz/Orbit")
        {
            manager_->getViewManager()->setCurrentViewControllerType("rviz/FPS");
            rviz::FPSViewController* fps_view_controller_ = (rviz::FPSViewController*)(manager_->getViewManager()->getCurrent());
            //ROS_ERROR(" position %f %f %f orientation %f %f %f %f", render_panel_->getCamera()->getPosition().x, render_panel_->getCamera()->getPosition().y, render_panel_->getCamera()->getPosition().z, render_panel_->getCamera()->getOrientation().x, render_panel_->getCamera()->getOrientation().y, render_panel_->getCamera()->getOrientation().z, render_panel_->getCamera()->getOrientation().w);
        }
        else if (manager_->getViewManager()->getCurrent()->getClassId() == "rviz/FPS")
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
