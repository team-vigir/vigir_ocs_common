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
#include <flor_ocs_msgs/TwoPoint.h>
#include <flor_ocs_msgs/OCSAugmentRegions.h>

#include "map_view.h"

namespace vigir_ocs
{
// Constructor for MapView.  This does most of the work of the class.
MapView::MapView( QWidget* parent )
    : OrthoView( NULL, "/world", "MapView", parent ),
      selection_tool_enabled_( true )
{
    // block sending left/right mouse events to rviz by default
    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
    //((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::KEY_PRESS_EVENT,true,Qt::NoModifier,Qt::NoButton);

    // Add support for selection
    selection_tool_ = manager_->getToolManager()->addTool( "rviz/ImageSelectionToolCustom" );
    QObject::connect(mouse_event_handler_, SIGNAL(mouseLeftButton(bool,int,int)), this, SLOT(enableSelectionTool(bool,int,int)));
    QObject::connect(this, SIGNAL(unHighlight()), selection_tool_, SLOT(unHighlight()));

    Q_EMIT unHighlight();

    // create publisher for for the gridmap request
    grid_map_request_pub_ = nh_.advertise<flor_perception_msgs::EnvironmentRegionRequest>( "/flor/worldmodel/ocs/gridmap_request", 1, false );

    // create publisher for augmentations on the grid map
    augment_grid_map_pub_ = nh_.advertise<flor_ocs_msgs::OCSAugmentRegions>( "/flor/worldmodel/ocs_augmented_regions", 1, false );

    // create publisher for the octomap request
    octomap_request_pub_ = nh_.advertise<flor_perception_msgs::EnvironmentRegionRequest>( "/flor/worldmodel/ocs/octomap_request", 1, false );

    // create publisher for point cloud
    point_cloud_request_pub_ = nh_.advertise<flor_perception_msgs::PointCloudTypeRegionRequest>( "/flor/worldmodel/ocs/cloud_request", 1, false );

    // connect to selection display to query position/raycast
    QObject::connect(this, SIGNAL(queryPosition(int,int,Ogre::Vector3&)), selection_3d_display_, SLOT(queryPosition(int,int,Ogre::Vector3&)));

    selected_area_[0] = 0;
    selected_area_[1] = 0;
    selected_area_[2] = 0;
    selected_area_[3] = 0;

    setViewPlane("XY");

    addContextMenu();

    //set default tool
    manager_->getToolManager()->setCurrentTool( interactive_markers_tool_);
}

// Destructor.
MapView::~MapView()
{
}

void MapView::enableSelectionTool(bool activate, int x, int y)
{
    if(!setting_pose_)
    {
        if(selection_tool_enabled_)
        {
            std::cout << "selection tool: " << activate << std::endl;
            if(activate)
            {
                //if over a interactive marker dont change tool, allow interative marker manipulaion                            //lock to access weak ptr
                if(((rviz::InteractiveObjectWPtr)((rviz::InteractionToolCustom *)interactive_markers_tool_)->getCurrentObject()).lock() == NULL)
                {
                    selected_area_[0] = x;
                    selected_area_[1] = y;
                    // change to the selection tool and unblock events
                    manager_->getToolManager()->setCurrentTool( selection_tool_ );
                    //resetEventFilters();
                    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::RightButton);
                    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::RightButton);
                    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::RightButton);
                }
                else
                {
                    //resetEventFilters();
                    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::RightButton);
                    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::RightButton);
                    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::RightButton);
                }
            }
            else
            {
                selected_area_[2] = x;
                selected_area_[3] = y;
                // block again and change back
                //resetEventFilters();
                ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
                ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
                ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
                manager_->getToolManager()->setCurrentTool( interactive_markers_tool_);
            }
        }
        else
        {
            if(activate)
            {
                // unblock events if clicked
               // resetEventFilters();
                ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::RightButton);
                ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::RightButton);
                ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::RightButton);
            }
            else
            {
                // block again on release
                //resetEventFilters();
                ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
                ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
                ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
            }
        }
    }
}

void MapView::selectionToolToggle(bool enable)
{
    selection_tool_enabled_ = enable;
}

void MapView::requestMap(double min_z, double max_z, double resolution)
{
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    Ogre::Vector3 min, max;
    Q_EMIT queryPosition(selected_area_[0],selected_area_[1],min);
    Q_EMIT queryPosition(selected_area_[2],selected_area_[3],max);

    flor_perception_msgs::EnvironmentRegionRequest cmd;

    cmd.bounding_box_min.x = min.x;
    cmd.bounding_box_min.y = min.y;
    cmd.bounding_box_min.z = min_z;

    cmd.bounding_box_max.x = max.x;
    cmd.bounding_box_max.y = max.y;
    cmd.bounding_box_max.z = max_z;

    cmd.resolution = resolution;

    cmd.request_augment = 1;

    grid_map_request_pub_.publish(cmd);

    Q_EMIT unHighlight();
}

void MapView::requestOctomap(double min_z, double max_z, double resolution)
{
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    Ogre::Vector3 min, max;
    Q_EMIT queryPosition(selected_area_[0],selected_area_[1],min);
    Q_EMIT queryPosition(selected_area_[2],selected_area_[3],max);

    flor_perception_msgs::EnvironmentRegionRequest cmd;

    cmd.bounding_box_min.x = min.x;
    cmd.bounding_box_min.y = min.y;
    cmd.bounding_box_min.z = min_z;

    cmd.bounding_box_max.x = max.x;
    cmd.bounding_box_max.y = max.y;
    cmd.bounding_box_max.z = max_z;

    cmd.resolution = resolution;

    octomap_request_pub_.publish(cmd);

    Q_EMIT unHighlight();
}

bool MapView::hasValidSelection()
{
    // need to check if selection is within bounds
    //float win_width = render_panel_->width();
    //float win_height = render_panel_->height();

    int min_x, min_y, max_x, max_y;
    min_x = std::min(selected_area_[0],selected_area_[2]);
    min_y = std::min(selected_area_[1],selected_area_[3]);
    max_x = std::max(selected_area_[0],selected_area_[2]);
    max_y = std::max(selected_area_[1],selected_area_[3]);

    if(((max_x-min_x)*(max_y-min_y)) < 9) // if area of selection is too small to actually be a selection (less than 9 pixels), return false
        return false;

    return true;
}

void MapView::requestPointCloud(int type)
{
    flor_perception_msgs::PointCloudTypeRegionRequest cmd;
    cmd.environment_region_request = last_request_.environment_region_request;
    cmd.data_source = type;
    point_cloud_request_pub_.publish(cmd);
}

void MapView::requestPointCloud(double min_z, double max_z, double resolution, int type, int aggregation_size)
{
	// first we update the size of the points in the point cloud based on resolution
	if(type == flor_perception_msgs::PointCloudTypeRegionRequest::STEREO) // hard coded for now 
	    stereo_point_cloud_viewer_->subProp( "Size (m)" )->setValue( resolution );
	else if(type == flor_perception_msgs::PointCloudTypeRegionRequest::LIDAR_FILTERED || type == flor_perception_msgs::PointCloudTypeRegionRequest::LIDAR_UNFILTERED)
        region_point_cloud_viewer_->subProp( "Size (m)" )->setValue( resolution );

	// then we create the point cloud request message.
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    Ogre::Vector3 min, max;
    Q_EMIT queryPosition(selected_area_[0],selected_area_[1],min);
    Q_EMIT queryPosition(selected_area_[2],selected_area_[3],max);

    flor_perception_msgs::PointCloudTypeRegionRequest cmd;

    flor_perception_msgs::EnvironmentRegionRequest& env = cmd.environment_region_request;

    env.bounding_box_min.x = min.x;
    env.bounding_box_min.y = min.y;
    env.bounding_box_min.z = min_z;

    env.bounding_box_max.x = max.x;
    env.bounding_box_max.y = max.y;
    env.bounding_box_max.z = max_z;

    env.resolution = resolution;

    cmd.data_source = type;

    if (aggregation_size >= 0){
      cmd.aggregation_size = static_cast<unsigned int>(aggregation_size);
    }else{
      cmd.aggregation_size = 0;
    }

    point_cloud_request_pub_.publish(cmd);

    last_request_ = cmd;

    Q_EMIT unHighlight();
}

void MapView::addContextMenu()
{
    //can tell context menu to add a separator when this item is added
    contextMenuItem * separator = new contextMenuItem();
    separator->name = "Separator";

    //request point cloud from tools section
    vigir_ocs::Base3DView::makeContextChild("Request Point Cloud",boost::bind(&MapView::publishPointCloudWorldRequest,this), NULL, contextMenuElements);

    contextMenuElements.push_back(separator);

    vigir_ocs::Base3DView::makeContextChild("Define Target Pose - Walk",boost::bind(&MapView::defineWalkPosePressed,this), NULL, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Define Target Pose - Step",boost::bind(&MapView::defineStepPosePressed,this), NULL, contextMenuElements);

    contextMenuElements.push_back(separator);

    vigir_ocs::Base3DView::makeContextChild("Request Area Map",boost::bind(&MapView::requestAreaMapContext,this), NULL, contextMenuElements);

    contextMenuElements.push_back(separator);

    vigir_ocs::Base3DView::makeContextChild("Request Octomap",boost::bind(&MapView::requestOctomapContext,this), NULL, contextMenuElements);

    pointCloudMenu = vigir_ocs::Base3DView::makeContextParent("Request Point Clound Types", contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("LIDAR filtered",boost::bind(&MapView::requestPointCloud,this,0), pointCloudMenu, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("LIDAR unfiltered",boost::bind(&MapView::requestPointCloud,this,1), pointCloudMenu, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Stereo",boost::bind(&MapView::requestPointCloud,this,2), pointCloudMenu, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Stereo Sandia",boost::bind(&MapView::requestPointCloud,this,3), pointCloudMenu, contextMenuElements);

    contextMenuElements.push_back(separator);

    blockRegion = vigir_ocs::Base3DView::makeContextParent("Block Region", contextMenuElements);

    vigir_ocs::Base3DView::makeContextChild("Axis-Aligned Rectangle",boost::bind(&MapView::blockRegionContext,this, 1), blockRegion, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Line",boost::bind(&MapView::blockRegionContext,this, 0), blockRegion, contextMenuElements);

    clearRegion = vigir_ocs::Base3DView::makeContextParent("Clear Region", contextMenuElements);

    vigir_ocs::Base3DView::makeContextChild("Axis-Aligned Rectangle",boost::bind(&MapView::clearRegionContext,this, 1), clearRegion, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Line",boost::bind(&MapView::clearRegionContext,this, 0), clearRegion, contextMenuElements);

    contextMenuElements.push_back(separator);

    //add all context menu items to each view
    for(int i=0;i<contextMenuElements.size();i++)
    {
        this->addToContextVector(contextMenuElements[i]);
    }
}

void MapView::requestAreaMapContext()
{
    Q_EMIT UIrequestAreaMap();
}

void MapView::requestOctomapContext()
{
    Q_EMIT UIrequestOctomap();
}

void MapView::blockRegionContext(int boxType)
{
    flor_ocs_msgs::OCSAugmentRegions augmentation;
    augmentation.header.frame_id = base_frame_;
    augmentation.map_selection = 2;
    Ogre::Vector3 min, max;
    Q_EMIT queryPosition(selected_area_[0],selected_area_[1],min);
    Q_EMIT queryPosition(selected_area_[2],selected_area_[3],max);
    flor_ocs_msgs::TwoPoint box;
    box.min[0] = min.x;
    box.min[1] = min.y;
    box.max[0] = max.x;
    box.max[1] = max.y;
    box.type = boxType;
    augmentation.blocked.push_back(box);
    augment_grid_map_pub_.publish(augmentation);
}

void MapView::clearRegionContext(int boxType)
{
    flor_ocs_msgs::OCSAugmentRegions augmentation;
    augmentation.header.frame_id = base_frame_;
    augmentation.map_selection = 2;
    Ogre::Vector3 min, max;
    Q_EMIT queryPosition(selected_area_[0],selected_area_[1],min);
    Q_EMIT queryPosition(selected_area_[2],selected_area_[3],max);
    flor_ocs_msgs::TwoPoint box;
    box.min[0] = min.x;
    box.min[1] = min.y;
    box.max[0] = max.x;
    box.max[1] = max.y;
    box.type = boxType;
    augmentation.cleared.push_back(box);
    augment_grid_map_pub_.publish(augmentation);
}

//void MapView::createContextMenu(bool, int x, int y)
//{
//    initializing_context_menu_++;

//    OrthoView::createContextMenu(false, x, y);

//    // create sub-menus
//    QMenu block_region_menu("Block Region");
//    block_region_menu.addAction("Axis-Aligned Rectangle");
//    block_region_menu.addAction("Line");
//    context_menu_.addMenu(&block_region_menu);
//    QMenu clear_region_menu("Clear Region");
//    clear_region_menu.addAction("Axis-Aligned Rectangle");
//    clear_region_menu.addAction("Line");
//    context_menu_.addMenu(&clear_region_menu);


//    if(initializing_context_menu_ == 1)
//        processContextMenu(x, y);

//    initializing_context_menu_--;
//}

//void MapView::processContextMenu(int x, int y)
//{
//    Base3DView::processContextMenu(x, y);

//    if(context_menu_selected_item_ != NULL)
//    {
//        ROS_INFO("%s->%s",((QMenu*)context_menu_selected_item_->parent())->title().toStdString().c_str(),context_menu_selected_item_->text().toStdString().c_str());
//        if(((QMenu*)context_menu_selected_item_->parent())->title() == QString("Block Region") &&
//           context_menu_selected_item_->text() == QString("Axis-Aligned Rectangle"))
//        {
//            flor_ocs_msgs::OCSAugmentRegions augmentation;
//            augmentation.header.frame_id = base_frame_;
//            augmentation.map_selection = 2;
//            Ogre::Vector3 min, max;
//            Q_EMIT queryPosition(selected_area_[0],selected_area_[1],min);
//            Q_EMIT queryPosition(selected_area_[2],selected_area_[3],max);
//            flor_ocs_msgs::TwoPoint box;
//            box.min[0] = min.x;
//            box.min[1] = min.y;
//            box.max[0] = max.x;
//            box.max[1] = max.y;
//            box.type = 1;
//            augmentation.blocked.push_back(box);
//            augment_grid_map_pub_.publish(augmentation);
//        }
//        else if(((QMenu*)context_menu_selected_item_->parent())->title() == QString("Block Region") &&
//                context_menu_selected_item_->text() == QString("Line"))
//        {
//            flor_ocs_msgs::OCSAugmentRegions augmentation;
//            augmentation.header.frame_id = base_frame_;
//            augmentation.map_selection = 2;
//            Ogre::Vector3 min, max;
//            Q_EMIT queryPosition(selected_area_[0],selected_area_[1],min);
//            Q_EMIT queryPosition(selected_area_[2],selected_area_[3],max);
//            ROS_ERROR("%f %f -> %f %f",min.x,min.y,max.x,max.y);
//            flor_ocs_msgs::TwoPoint box;
//            box.min[0] = min.x;
//            box.min[1] = min.y;
//            box.max[0] = max.x;
//            box.max[1] = max.y;
//            box.type = 0;
//            augmentation.blocked.push_back(box);
//            augment_grid_map_pub_.publish(augmentation);
//        }
//        else if(((QMenu*)context_menu_selected_item_->parent())->title() == QString("Clear Region") &&
//                context_menu_selected_item_->text() == QString("Axis-Aligned Rectangle"))
//        {
//            flor_ocs_msgs::OCSAugmentRegions augmentation;
//            augmentation.header.frame_id = base_frame_;
//            augmentation.map_selection = 2;
//            Ogre::Vector3 min, max;
//            Q_EMIT queryPosition(selected_area_[0],selected_area_[1],min);
//            Q_EMIT queryPosition(selected_area_[2],selected_area_[3],max);
//            flor_ocs_msgs::TwoPoint box;
//            box.min[0] = min.x;
//            box.min[1] = min.y;
//            box.max[0] = max.x;
//            box.max[1] = max.y;
//            box.type = 1;
//            augmentation.cleared.push_back(box);
//            augment_grid_map_pub_.publish(augmentation);
//        }
//        else if(((QMenu*)context_menu_selected_item_->parent())->title() == QString("Clear Region") &&
//                context_menu_selected_item_->text() == QString("Line"))
//        {
//            flor_ocs_msgs::OCSAugmentRegions augmentation;
//            augmentation.header.frame_id = base_frame_;
//            augmentation.map_selection = 2;
//            Ogre::Vector3 min, max;
//            Q_EMIT queryPosition(selected_area_[0],selected_area_[1],min);
//            Q_EMIT queryPosition(selected_area_[2],selected_area_[3],max);
//            flor_ocs_msgs::TwoPoint box;
//            box.min[0] = min.x;
//            box.min[1] = min.y;
//            box.max[0] = max.x;
//            box.max[1] = max.y;
//            box.type = 0;
//            augmentation.cleared.push_back(box);
//            augment_grid_map_pub_.publish(augmentation);

//        }
//    }
//}

}

