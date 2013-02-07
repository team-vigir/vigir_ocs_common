/* 
 * PointCloudViewer class implementation.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/06/2012):
 * - got it working properly, tested with gazebo 1.3 and pr2
 * - does not seem to work with DRC sim: is it not being sent by the robot?
 */

#include <QLabel>
#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "point_cloud_viewer.h"

// Constructor for PointCloudViewer.  This does most of the work of the class.
PointCloudViewer::PointCloudViewer( QWidget* parent )
  : QWidget( parent )
{
  // Create a new label for this widget.
  QLabel* image_label = new QLabel( "rviz/PointCloud2" );
  
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

  // Create a point cloud display.
  point_cloud_viewer_ = manager_->createDisplay( "rviz/PointCloud2", "Point cloud", true );
  ROS_ASSERT( point_cloud_viewer_ != NULL );

  // Set image topic
  point_cloud_viewer_->subProp( "Style" )->setValue( "Points" );
  point_cloud_viewer_->subProp( "Topic" )->setValue( "/scan_cloud_filtered" );
  point_cloud_viewer_->subProp( "Size (m)" )->setValue( 0.1 );
  point_cloud_viewer_->subProp( "Decay Time" )->setValue( 0 );

  // Set topic that will be used as 0,0,0 -> reference for all the other transforms
  // IMPORTANT: WITHOUT THIS, ALL THE DIFFERENT PARTS OF THE ROBOT MODEL WILL BE DISPLAYED AT 0,0,0
  manager_->getFrameManager()->setFixedFrame("/pelvis");
}

// Destructor.
PointCloudViewer::~PointCloudViewer()
{
  delete manager_;
}

