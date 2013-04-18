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
// Constructor for CameraViewerCustom.  This does most of the work of the class.

QPushButton* toggleButton;
int selectedArea[4];
CameraViewerCustom::CameraViewerCustom( QWidget* parent )
  : QWidget( parent )
{
  // Create a new label for this widget.
  //QLabel* image_label = new QLabel( "rviz/ImageCustom, topic /multisense_sl/left/image_raw" );
  //    this->setMouseTracking(true);
  //  selectedArea = new int[4];
  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  //main_layout->addWidget( image_label );
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Make signal/slot connections.

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->setFixedFrame("/pelvis");
  manager_->initialize();
  manager_->startUpdate();

  // Create a camera/image display.
  // can be both a camera or an image, the difference is that the camera has 3D content with it
  //camera_viewer_ = manager_->createDisplay( "rviz/Camera", "Camera image", true );
  camera_viewer_ = manager_->createDisplay( "rviz/CameraDisplayCustom", "Camera image", true ); // this would use the plugin instead of manually adding the display object to the manager
  //camera_viewer_ = new rviz::ImageDisplayCustom(); // -> need to make this class failsafe when render_panel_ is null so that I can use the createDisplay function and a signal to set the render panel
  ((rviz::CameraDisplayCustom*)camera_viewer_)->setRenderPanel( render_panel_ );

  //manager_->addDisplay( camera_viewer_, true );//manager_->createDisplay( "rviz/ImageCustom", "Camera image", true );
  //camera_viewer_->setName( "Camera Image" );
  ROS_ASSERT( camera_viewer_ != NULL );



  // Add support for selection
  //selection_tool_ = manager_->getToolManager()->addTool( "rviz/Select" );
  selection_tool_ = manager_->getToolManager()->addTool( "rviz/ImageSelectionToolCustom" );

  manager_->getToolManager()->setCurrentTool( selection_tool_ );

  // connect the selection tool select signal to this
  QObject::connect(selection_tool_, SIGNAL(select(int,int,int,int)), this, SLOT(select(int,int,int,int)));

  // Set image topic
  //camera_viewer_->subProp( "Image Topic" )->setValue( "/multisense_sl/camera/left/image_raw" );
  camera_viewer_->subProp( "Image Topic" )->setValue( "/l_image_full/image_raw" );
  camera_viewer_->setEnabled(false);
  camera_viewer_->setEnabled(true);


  // Create a RobotModel display.
    robot_model_ = manager_->createDisplay( "rviz/RobotModel", "Robot model", false );
    ROS_ASSERT( robot_model_ != NULL );

  // Create a LaserScan display.
    laser_scan_ = manager_->createDisplay( "rviz/LaserScan", "Laser Scan", false );
    ROS_ASSERT( laser_scan_ != NULL );
    laser_scan_->subProp( "Style" )->setValue( "Points" );
    laser_scan_->subProp( "Topic" )->setValue( "/multisense_sl/laser/scan" );
    laser_scan_->subProp( "Size (m)" )->setValue( 0.1 );
    laser_scan_->subProp( "Decay Time" )->setValue( 1 );


    // Create a MarkerArray display.
    marker_array_ = manager_->createDisplay( "rviz/MarkerArray", "MarkerArray", false );
    ROS_ASSERT( marker_array_ != NULL );

    marker_array_->subProp( "Marker Topic" )->setValue( "/worldmodel_main/occupied_cells_vis_array" );


    // Create a point cloud display.
    stereo_point_cloud_viewer_ = manager_->createDisplay( "rviz/PointCloud2", "Point Cloud", false );
    ROS_ASSERT( stereo_point_cloud_viewer_ != NULL );
    stereo_point_cloud_viewer_->subProp( "Style" )->setValue( "Points" );
    stereo_point_cloud_viewer_->subProp( "Topic" )->setValue( "/multisense_sl/camera/points2" );
    stereo_point_cloud_viewer_->subProp( "Size (Pixels)" )->setValue( 3 );


    lidar_point_cloud_viewer_ = manager_->createDisplay( "rviz/PointCloud2", "Point Cloud", false );
    ROS_ASSERT( lidar_point_cloud_viewer_ != NULL );
    lidar_point_cloud_viewer_->subProp( "Style" )->setValue( "Points" );
    lidar_point_cloud_viewer_->subProp( "Topic" )->setValue( "/scan_cloud_filtered" );
    lidar_point_cloud_viewer_->subProp( "Size (Pixels)" )->setValue( 3 );

   ((rviz::CameraDisplayCustom*)camera_viewer_)->setup( );

    manager_->getFrameManager()->setFixedFrame("/pelvis");
  // ((rviz::CameraDisplayCustom*)camera_viewer_)->addManager(manager_ );
    select_manager_ = manager_->getSelectionManager();
    select_manager_->setTextureSize(512);

  //  QWidget* parentW = this->parentWidget()->findChild<QWidget *>("widget_6");

    //toggleButton = parentW->findChild<QPushButton *>("toggleButton");
  //  toggleButton->setEnabled(false);
   //connect(toggleButton, SIGNAL(clicked()), this, SLOT(disableSelection()));
 // manager_->getFrameManager()->setFixedFrame("/");

  QObject::connect(this, SIGNAL(setFullImageResolution(int)), camera_viewer_, SLOT(changeFullImageResolution(int)));
  QObject::connect(this, SIGNAL(setCropImageResolution(int)), camera_viewer_, SLOT(changeCropImageResolution(int)));
  QObject::connect(this, SIGNAL(setCameraSpeed(int)), camera_viewer_, SLOT(changeCameraSpeed(int)));
}

// Destructor.
CameraViewerCustom::~CameraViewerCustom()
{
  delete manager_;
}

void CameraViewerCustom::select( int x1, int y1, int x2, int y2 )
{
    selectedArea[0] = x1;
    selectedArea[1] = y1;
    selectedArea[2] = x2;
    selectedArea[3] = y2;
   // select_manager_->highlight(, x1, y1, x2, y2 );
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
    ((rviz::CameraDisplayCustom*)camera_viewer_)->selectionProcessed( selectedArea[0], selectedArea[1], selectedArea[2], selectedArea[3] );
    select_manager_->removeHighlight();

 //   ((rviz::CameraDisplayCustom*)camera_viewer_)->selectionProcessed( 0, 0, 0, 0 );

}

void CameraViewerCustom::robotModelToggled( bool selected )
{
    robot_model_->setEnabled( selected );
}

void CameraViewerCustom::lidarPointCloudToggled( bool selected )
{
    lidar_point_cloud_viewer_->setEnabled( selected );
}

void CameraViewerCustom::stereoPointCloudToggled( bool selected )
{
    stereo_point_cloud_viewer_->setEnabled( selected );
}

void CameraViewerCustom::laserScanToggled( bool selected )
{
    laser_scan_->setEnabled( selected );
}

void CameraViewerCustom::markerArrayToggled( bool selected )
{
    marker_array_->setEnabled( selected );
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

void QWidget::mouseMoveEvent(QMouseEvent *event)
{
    QPoint point = event->pos();
    std::cout<<"Prints when mouse is moved";
    if(((point.x()<selectedArea[0] && point.x()>selectedArea[2]) ||
       (point.x()>selectedArea[0] && point.x()<selectedArea[2])) &&
       ((point.y()<selectedArea[1] && point.y()>selectedArea[3]) ||
       (point.y()>selectedArea[1] && point.y()<selectedArea[3])))
    {
    }

}

