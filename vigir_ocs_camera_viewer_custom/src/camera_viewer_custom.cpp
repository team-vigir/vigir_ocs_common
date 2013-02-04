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

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "rviz/tool_manager.h"
#include "camera_viewer_custom.h"
#include "image_display_custom.h"

// Constructor for CameraViewerCustom.  This does most of the work of the class.
CameraViewerCustom::CameraViewerCustom( QWidget* parent )
  : QWidget( parent )
{
  // Create a new label for this widget.
  //QLabel* image_label = new QLabel( "rviz/ImageCustom, topic /multisense_sl/left/image_raw" );
  
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
  manager_->initialize();
  manager_->startUpdate();

  // Create a camera/image display.
  // can be both a camera or an image, the difference is that the camera has 3D content with it
  //camera_viewer_ = manager_->createDisplay( "rviz/Camera", "Camera image", true );
  //camera_viewer_ = manager_->createDisplay( "vigir_ocs_rviz_plugins/ImageDisplayCustom", "Camera image", true ); // this would use the plugin instead of manually adding the display object to the manager
  camera_viewer_ = new rviz::ImageDisplayCustom(); // -> need to make this class failsafe when render_panel_ is null so that I can use the createDisplay function and a signal to set the render panel
  ((rviz::ImageDisplayCustom*)camera_viewer_)->setRenderPanel( render_panel_ );
  manager_->addDisplay( camera_viewer_, true );//manager_->createDisplay( "rviz/ImageCustom", "Camera image", true );
  camera_viewer_->setName( "Camera Image" );
  ROS_ASSERT( camera_viewer_ != NULL );

  // Add support for selection
  //selection_tool_ = manager_->getToolManager()->addTool( "rviz/Select" );
  selection_tool_ = manager_->getToolManager()->addTool( "rviz/SelectionToolCustom" );

  manager_->getToolManager()->setCurrentTool( selection_tool_ );

  // connect the selection tool select signal to this
  QObject::connect(selection_tool_, SIGNAL(select(int,int,int,int)), this, SLOT(select(int,int,int,int)));

  // Set image topic
  //camera_viewer_->subProp( "Image Topic" )->setValue( "/right_eye/image_raw" );
  camera_viewer_->subProp( "Image Topic" )->setValue( "/multisense_sl/left/image_raw" );
}

// Destructor.
CameraViewerCustom::~CameraViewerCustom()
{
  delete manager_;
}

void CameraViewerCustom::select( int x1, int y1, int x2, int y2 )
{
  ((rviz::ImageDisplayCustom*)camera_viewer_)->selectionProcessed( x1, y1, x2, y2 );
}
