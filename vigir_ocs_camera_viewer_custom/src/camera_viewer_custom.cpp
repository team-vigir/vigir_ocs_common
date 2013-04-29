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
#include <QPushButton>
#include <QMouseEvent>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "rviz/tool_manager.h"
#include "camera_viewer_custom.h"
//#include "image_display_custom.h"
#include "camera_display_custom.h"
#include "image_selection_tool_custom.h"

// Constructor for CameraViewerCustom.  This does most of the work of the class.

QPushButton* toggleButton;
QPushButton* xButton;

int selectedArea[4];

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

    // Add support for selection
    selection_tool_ = manager_->getToolManager()->addTool( "rviz/ImageSelectionToolCustom" );

    manager_->getToolManager()->setCurrentTool( selection_tool_ );

    // connect the selection tool select signal to this
    QObject::connect(selection_tool_, SIGNAL(select(int,int,int,int)), this, SLOT(select(int,int,int,int)));

    // Set image topic
    camera_viewer_->subProp( "Image Topic" )->setValue( "/l_image_full/image_raw" );
    camera_viewer_->setEnabled(false);
    camera_viewer_->setEnabled(true);

    select_manager_ = manager_->getSelectionManager();
    select_manager_->setTextureSize(512);

    QObject::connect(this, SIGNAL(setFullImageResolution(int)), camera_viewer_, SLOT(changeFullImageResolution(int)));
    QObject::connect(this, SIGNAL(setCropImageResolution(int)), camera_viewer_, SLOT(changeCropImageResolution(int)));
    QObject::connect(this, SIGNAL(setCameraSpeed(int)), camera_viewer_, SLOT(changeCameraSpeed(int)));

    // and advertise the head pitch update function
    head_pitch_update_pub_ = n_.advertise<std_msgs::Float64>( "/atlas/pos_cmd/neck_ay", 1, false );
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
    /** std::cout<<"selected x1 " <<x1<<std::endl;
    std::cout<<"selected y1 " <<y1<<std::endl;
    std::cout<<"selected x2 " <<x2<<std::endl;
    std::cout<<"selected y2 " <<y2<<std::endl;**/

    select_manager_->highlight(render_panel_->getViewport(), x1, y1, x2, y2 );
    /** if(!disabled)
    {
        ((rviz::CameraDisplayCustom*)camera_viewer_)->selectionProcessed( x1, y1, x2, y2 );
    //    toggleButton->setEnabled(true);
        disabled = true;
    }**/
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
    //  toggleButton->setEnabled(false);
    //  okay = true;
    ((rviz::ImageSelectionToolCustom*)selection_tool_)->unHighlight();
    ((rviz::CameraDisplayCustom*)camera_viewer_)->selectionProcessed( selectedArea[0], selectedArea[1], selectedArea[2], selectedArea[3] );

    //   ((rviz::CameraDisplayCustom*)camera_viewer_)->selectionProcessed( 0, 0, 0, 0 );

}

void CameraViewerCustom::changeAlpha(int newAlpha)
{
    ((rviz::CameraDisplayCustom*)camera_viewer_)->setAlpha(1.0f-(newAlpha/100.0f));
}

void CameraViewerCustom::changeLayer(int selectedLayer)
{

    ((rviz::CameraDisplayCustom*)camera_viewer_)->setLayer(selectedLayer);

}

void CameraViewerCustom::changeZoom(int newZoom)
{
    ((rviz::CameraDisplayCustom*)camera_viewer_)->setZoom((newZoom/100000.0f));

}

void CameraViewerCustom::mouseMoveEvent(QMouseEvent *event)
{
    /**  QPoint point = event->pos();
    std::cout<<"Prints when mouse is moved"<<std::endl;
    int x = this->size().width();
    int y = this->size().height();
    std::cout<<"width is "<<x<<std::endl;
    std::cout<<"height is "<<y<<std::endl;


    if(((point.x()<selectedArea[0] && point.x()>selectedArea[2]) ||
       (point.x()>selectedArea[0] && point.x()<selectedArea[2])) &&
       ((point.y()<selectedArea[1] && point.y()>selectedArea[3]) ||
       (point.y()>selectedArea[1] && point.y()<selectedArea[3])))
    {
    //    xButton->show();
    }
    else
    {
    //    xButton->hide();
    }**/

}
}

