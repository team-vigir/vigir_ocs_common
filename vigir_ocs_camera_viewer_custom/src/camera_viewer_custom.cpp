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
#include <QPushButton>

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
#include "rviz/viewport_mouse_event.h"

// Constructor for CameraViewerCustom.  This does most of the work of the class.

QPushButton* xButton;
bool selectionMade = false;
namespace vigir_ocs
{
CameraViewerCustom::CameraViewerCustom( QWidget* parent )
    : Base3DView( "/world", parent ),
      selected_topic(0)
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
    ghost_robot_model_->setEnabled(false);

    QObject::connect(this, SIGNAL(setFullImageResolution(int)), camera_viewer_, SLOT(changeFullImageResolution(int)));
    QObject::connect(this, SIGNAL(setCropImageResolution(int)), camera_viewer_, SLOT(changeCropImageResolution(int)));
    QObject::connect(this, SIGNAL(setCameraSpeed(int)), camera_viewer_, SLOT(changeCameraSpeed(int)));
    QObject::connect(this, SIGNAL(setCropCameraSpeed(int)), camera_viewer_, SLOT(changeCropCameraSpeed(int)));

    QObject::connect(this, SIGNAL(unHighlight()), selection_tool_, SLOT(unHighlight()));

    // and advertise the head pitch update function
    head_pitch_update_pub_ = n_.advertise<std_msgs::Float64>( "/atlas/pos_cmd/neck_ay", 1, false );

    rviz::EmptyViewController* camera_controller = new rviz::EmptyViewController();
    camera_controller->initialize( render_panel_->getManager() );
    render_panel_->setViewController( camera_controller );

    xButton = new QPushButton("X",this);
    QObject::connect(xButton, SIGNAL(clicked()), this, SLOT(closeSelectedArea()));
    xButton->hide();
    QObject::connect((selection_tool_), SIGNAL(mouseHasMoved(int,int)), this, SLOT(mouseMoved(int,int)));

    Q_EMIT setMarkerScale(0.001f);

    // advertise pointcloud request
    pointcloud_request_pub_ = n_.advertise<geometry_msgs::PointStamped>( "/flor/worldmodel/ocs/dist_query_pointcloud_request", 1, false );
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
    selected_topic = t;

    std::cout << "Camera topic changed:" << selected_topic << std::endl;
    switch( selected_topic )
    {
    case 0: // Head Left
    {
        camera_viewer_->subProp( "Image Request Topic" )->setValue( "/l_image_full/image_request" );
        camera_viewer_->subProp( "Image Topic" )->setValue( "/l_image_full/image_raw" );
        camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( "/l_image_cropped/image_request" );
        camera_viewer_->subProp( "Cropped Image Topic" )->setValue( "/l_image_cropped/image_raw" );
        break;
    }
    case 1: // Head Right
    {
        camera_viewer_->subProp( "Image Request Topic" )->setValue( "/r_image_full/image_request" );
        camera_viewer_->subProp( "Image Topic" )->setValue( "/r_image_full/image_raw" );
        camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( "/r_image_cropped/image_request" );
        camera_viewer_->subProp( "Cropped Image Topic" )->setValue( "/r_image_cropped/image_raw" );
        break;
    }
    case 2: // Left hand Left
    {
        camera_viewer_->subProp( "Image Request Topic" )->setValue( "/lhl_image_full/image_request" );
        camera_viewer_->subProp( "Image Topic" )->setValue( "/lhl_image_full/image_raw" );
        camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( "/lhl_image_cropped/image_request" );
        camera_viewer_->subProp( "Cropped Image Topic" )->setValue( "/lhl_image_cropped/image_raw" );
        break;
    }
    case 3: // Left hand Right
    {
        camera_viewer_->subProp( "Image Request Topic" )->setValue( "/lhr_image_full/image_request" );
        camera_viewer_->subProp( "Image Topic" )->setValue( "/lhr_image_full/image_raw" );
        camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( "/lhr_image_cropped/image_request" );
        camera_viewer_->subProp( "Cropped Image Topic" )->setValue( "/lhr_image_cropped/image_raw" );
        break;
    }
    case 4: // Right hand Left
    {
        camera_viewer_->subProp( "Image Request Topic" )->setValue( "/rhl_image_full/image_request" );
        camera_viewer_->subProp( "Image Topic" )->setValue( "/rhl_image_full/image_raw" );
        camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( "/rhl_image_cropped/image_request" );
        camera_viewer_->subProp( "Cropped Image Topic" )->setValue( "/rhl_image_cropped/image_raw" );
        break;
    }
    case 5: // Right hand Right
    {
        camera_viewer_->subProp( "Image Request Topic" )->setValue( "/rhr_image_full/image_request" );
        camera_viewer_->subProp( "Image Topic" )->setValue( "/rhr_image_full/image_raw" );
        camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( "/rhr_image_cropped/image_request" );
        camera_viewer_->subProp( "Cropped Image Topic" )->setValue( "/rhr_image_cropped/image_raw" );
        break;
    }
    }
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
    std::cout << "changed camera update speed: " << t << std::endl;
    Q_EMIT setCameraSpeed( t );
}

void CameraViewerCustom::changeCropCameraSpeed( int t )
{
    Q_EMIT setCropCameraSpeed( t );
}


void CameraViewerCustom::disableSelection()
{
     //((rviz::ImageSelectionToolCustom*)selection_tool_)->unHighlight();
    Q_EMIT unHighlight();
 
  //  toggleButton->setEnabled(false);
  //  okay = true;
    ((rviz::CameraDisplayCustom*)camera_viewer_)->selectionProcessed( selectedArea[0], selectedArea[1], selectedArea[2], selectedArea[3] );
    selectionMade = true;
    int rightSide = 0;
    int topSide = 0;
    if(selectedArea[0]<selectedArea[2])
    {
        rightSide = selectedArea[2];
    }
    else
    {
        rightSide = selectedArea[0];
    }

    if(selectedArea[1]<selectedArea[3])
    {
        topSide = selectedArea[1];
    }
    else
    {
        topSide = selectedArea[3];
    }
   // xButton = new QPushButton(render_panel_);
    xButton->setGeometry(rightSide-10, topSide+10, 20,20);
  //  xButton->show();
 // connect(xButton, SIGNAL(clicked()), this, SLOT(closeSelectedArea()));
  //  xButton->x =rightSide;
 //   xButton->y = topSide;



 //   ((rviz::CameraDisplayCustom*)camera_viewer_)->selectionProcessed( 0, 0, 0, 0 );
}

void CameraViewerCustom::requestPointCloudROI()
{
    Q_EMIT unHighlight();

    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    Ogre::Ray mouseRay = this->render_panel_->getCamera()->getCameraToViewportRay(((float)(selectedArea[0]+selectedArea[2])/2.0f)/win_width, (float)((float)(selectedArea[1]+selectedArea[3])/2.0f)/win_height);
    Ogre::Vector3 direction = mouseRay.getDirection();

    geometry_msgs::PointStamped cmd;

    cmd.header.stamp = ros::Time::now();
    std::string camera_topic = "";
    switch(selected_topic)
    {
    case 0: camera_topic = "head left"; break;
    case 1: camera_topic = "head right"; break;
    case 2: camera_topic = "left hand left"; break;
    case 3: camera_topic = "left hand right"; break;
    case 4: camera_topic = "right hand left"; break;
    case 5: camera_topic = "right hand right"; break;
    }

    cmd.header.frame_id = camera_topic;

    cmd.point.x = direction.x;
    cmd.point.y = direction.y;
    cmd.point.z = direction.z;

    pointcloud_request_pub_.publish(cmd);
}

void CameraViewerCustom::changeAlpha(int newAlpha)
{
    ((rviz::CameraDisplayCustom*)camera_viewer_)->setAlpha(1.0f-(newAlpha/100.0f));
}

void CameraViewerCustom::mouseMoved(int newX, int newY)
{
   // QPoint point = event->pos();
  /**  std::cout<<"Prints when mouse is moved"<<std::endl;
    int x = this->size().width();
    int y = this->size().height();
    std::cout<<"width is "<<x<<std::endl;
    std::cout<<"height is "<<y<<std::endl;**/


    if(((newX<selectedArea[0] && newX>selectedArea[2]) ||
       (newX>selectedArea[0] && newX<selectedArea[2])) &&
       ((newY<selectedArea[1] && newY>selectedArea[3]) ||
       (newY>selectedArea[1] && newY<selectedArea[3])) && selectionMade)
    {
        xButton->show();
    }
    else
    {
        xButton->hide();
    }

}

void CameraViewerCustom::closeSelectedArea()
{
    std::cout<<"This gets hit"<<std::endl;
    ((rviz::CameraDisplayCustom*)camera_viewer_)->closeSelected();
    xButton->hide();
    selectionMade = false;
}
}

