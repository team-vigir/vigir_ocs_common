/*
 * CameraViewerCustom class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on librviz_tutorials.
 *
 * Latest changes (12/11/2012):
 */

#include <QLabel>
#include <QVBoxLayout>
#include <QMouseEvent>

#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "rviz/tool_manager.h"
#include "rviz/view_manager.h"
#include "camera_viewer_custom.h"
//#include "image_display_custom.h"
#include "camera_display_custom.h"
#include "image_selection_tool_custom.h"
#include "empty_view_controller.h"

// Constructor for CameraViewerCustom.  This does most of the work of the class.

namespace vigir_ocs
{
CameraViewerCustom::CameraViewerCustom( QWidget* parent )
    : Base3DView( "/pelvis", parent )
{
    // Create a camera/image display.
    camera_viewer_ = manager_->createDisplay( "rviz/CameraDisplayCustom", "Camera image", true ); // this would use the plugin instead of manually adding the display object to the manager
    ROS_ASSERT( camera_viewer_ != NULL );

    ((rviz::CameraDisplayCustom*)camera_viewer_)->setRenderPanel( render_panel_ );
    ((rviz::CameraDisplayCustom*)camera_viewer_)->setup();

    // Set image topic
    camera_viewer_->subProp( "Image Topic" )->setValue( "/l_image_full/image_raw" );
    camera_viewer_->setEnabled(false);
    camera_viewer_->setEnabled(true);

    // Add support for selection
    selection_tool_ = manager_->getToolManager()->addTool( "rviz/ImageSelectionToolCustom" );

    manager_->getToolManager()->setCurrentTool( selection_tool_ );

    // connect the selection tool select signal to this
    QObject::connect(selection_tool_, SIGNAL(select(int,int,int,int)), this, SLOT(select(int,int,int,int)));

    robot_model_->setEnabled(false);

    QObject::connect(this, SIGNAL(setFullImageResolution(int)), camera_viewer_, SLOT(changeFullImageResolution(int)));
    QObject::connect(this, SIGNAL(setCropImageResolution(int)), camera_viewer_, SLOT(changeCropImageResolution(int)));
    QObject::connect(this, SIGNAL(setCameraSpeed(int)), camera_viewer_, SLOT(changeCameraSpeed(int)));

    QObject::connect(this, SIGNAL(unHighlight()), selection_tool_, SLOT(unHighlight()));

    // and advertise the head pitch update function
    head_pitch_update_pub_ = n_.advertise<std_msgs::Float64>( "/atlas/pos_cmd/neck_ay", 1, false );

    rviz::EmptyViewController* camera_controller = new rviz::EmptyViewController();
    camera_controller->initialize( render_panel_->getManager() );
    render_panel_->setViewController( camera_controller );
}

// Destructor.
CameraViewerCustom::~CameraViewerCustom()
{
    delete manager_;
}

void CameraViewerCustom::setCameraPitch( int degrees )
{
    std_msgs::Float64 cmd;
    cmd.data = degrees*0.0174532925;
    head_pitch_update_pub_.publish(cmd);
}

void CameraViewerCustom::select( int x1, int y1, int x2, int y2 )
{
    selectedArea[0] = x1;
    selectedArea[1] = y1;
    selectedArea[2] = x2;
    selectedArea[3] = y2;

    //select_manager_->highlight(render_panel_->getViewport(), x1, y1, x2, y2 );
}

void CameraViewerCustom::changeCameraTopic( int t )
{
    std::cout << "Camera topic changed:" << t << std::endl;
    //switch( t )
    //{
    //    case 0:
    //
    //}
}

void CameraViewerCustom::changeFullImageResolution( int t )
{
    Q_EMIT setFullImageResolution( t );
}


void CameraViewerCustom::changeCropImageResolution( int t )
{
    Q_EMIT setCropImageResolution( t );
}


void CameraViewerCustom::changeCameraSpeed( int t )
{
    Q_EMIT setCameraSpeed( t );
}


void CameraViewerCustom::disableSelection()
{
    //((rviz::ImageSelectionToolCustom*)selection_tool_)->unHighlight();
    Q_EMIT unHighlight();
    ((rviz::CameraDisplayCustom*)camera_viewer_)->selectionProcessed( selectedArea[0], selectedArea[1], selectedArea[2], selectedArea[3] );
}

void CameraViewerCustom::changeAlpha(int newAlpha)
{
    ((rviz::CameraDisplayCustom*)camera_viewer_)->setAlpha(1.0f-(newAlpha/100.0f));
}

void CameraViewerCustom::changeLayer(int selectedLayer)
{

    ((rviz::CameraDisplayCustom*)camera_viewer_)->setLayer(selectedLayer);

}
}

