/* 
 * CameraViewer class implementation.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/11/2012):
 * - fixed segfault issues
 */

#include <QLabel>
#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "camera_viewer.h"

// Constructor for CameraViewer.  This does most of the work of the class.
CameraViewer::CameraViewer( QWidget* parent )
  : QWidget( parent )
{
  // Create a new label for this widget.
  QLabel* image_label = new QLabel( "rviz/Image, topic /multisense_sl/left/image_raw" );
  
  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( image_label );
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
  manager_->initialize();
  manager_->startUpdate();

  // Create a camera/image display.
  // can be both a camera or an image, the difference is that the camera has 3D content with it
  // camera_viewer_ = manager_->createDisplay( "rviz/Camera", "Camera image", true );
  camera_viewer_ = manager_->createDisplay( "rviz/Image", "Camera image", true );
  ROS_ASSERT( camera_viewer_ != NULL );

  // Set image topic
  //camera_viewer_->subProp( "Image Topic" )->setValue( "/right_eye/image_raw" );
  camera_viewer_->subProp( "Image Topic" )->setValue( "/multisense_sl/camera/left/image_raw" );
}

// Destructor.
CameraViewer::~CameraViewer()
{
  delete manager_;
}

