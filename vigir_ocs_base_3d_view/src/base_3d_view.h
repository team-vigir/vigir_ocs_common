/*
 * Base3DView class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials and the .
 * 
 * Latest changes (12/08/2012):
 * - created class
 */

#ifndef BASE_3D_VIEW_H
#define BASE_3D_VIEW_H

#include <QWidget>
#include <QMouseEvent>
#include <QLabel>
#include <QPushButton>
#include <QFrame>
#include <QLineEdit>
#include <QBasicTimer>
#include <QThread>
#include <QMenu>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreRay.h>

#include <OGRE/OgreSceneManager.h>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <mouse_event_handler.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <flor_interactive_marker_server_custom/interactive_marker_server_custom.h>
#include <flor_ocs_msgs/OCSGhostControl.h>
#include <flor_ocs_msgs/OCSInteractiveMarkerAdd.h>
#include <flor_ocs_msgs/OCSInteractiveMarkerUpdate.h>
#include <flor_perception_msgs/RaycastRequest.h>

#include <string>

namespace rviz
{
class Display;
class Tool;
class RenderPanel;
class RenderPanelCustom;
class VisualizationManager;
class ViewController;
class FrameManager;
}

class QLineEditSmall : public QLineEdit
{
    Q_OBJECT
public:
    QLineEditSmall( const QString & contents, QWidget* parent = 0 ) : QLineEdit(contents,parent) { size_ = QSize(-1,-1); }
    virtual ~QLineEditSmall() {}
    void setSizeHint(const QSize& size) { size_ = size; }
    QSize sizeHint() const { return size_; }
private:
    QSize size_;
};

namespace vigir_ocs
{

// Class "Main3DView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class Base3DView: public QWidget
{
    Q_OBJECT
public:
    Base3DView( rviz::VisualizationManager* context = NULL, std::string base_frame = "/pelvis", QWidget* parent = 0 );
    virtual ~Base3DView();

    void processNewMap(const nav_msgs::OccupancyGrid::ConstPtr& pose);
    void processNewSelection( const geometry_msgs::Point::ConstPtr& pose );
    void processPointCloud( const sensor_msgs::PointCloud2::ConstPtr& pc );
    void processLeftArmEndEffector( const geometry_msgs::PoseStamped::ConstPtr& pose );
    void processRightArmEndEffector( const geometry_msgs::PoseStamped::ConstPtr& pose );
    void processLeftGhostHandPose( const geometry_msgs::PoseStamped::ConstPtr& pose );
    void processRightGhostHandPose( const geometry_msgs::PoseStamped::ConstPtr& pose );
    void processGhostControlState( const flor_ocs_msgs::OCSGhostControl::ConstPtr& msg );
    void processJointStates( const sensor_msgs::JointState::ConstPtr& states );
    void processPelvisResetRequest( const std_msgs::Bool::ConstPtr& msg );
    void processSendPelvisToFootstepRequest( const std_msgs::Bool::ConstPtr& msg );

    void onMarkerFeedback( const flor_ocs_msgs::OCSInteractiveMarkerUpdate::ConstPtr& msg );//std::string topic_name, geometry_msgs::PoseStamped pose);

    rviz::VisualizationManager* getVisualizationManager() { return manager_; }

    void updateRenderMask( bool );
public Q_SLOTS:
    // displays
    void robotModelToggled( bool );
    void graspModelToggled( bool );
    void templatesToggled( bool );
    void requestedPointCloudToggled( bool );
    void lidarPointCloudToggled( bool );
    void stereoPointCloudToggled( bool );
    void laserScanToggled( bool );
    void markerArrayToggled( bool );
    void gridMapToggled( bool );
    void footstepPlanningToggled( bool );
    void simulationRobotToggled( bool );
    // tools
    void cameraToggled( bool );
    void selectToggled( bool );
    void select3DToggled( bool );
    void markerRobotToggled( bool );
    void markerTemplateToggled( bool );
    virtual void defineWalkPosePressed();
    virtual void defineStepPosePressed();

    void newSelection( Ogre::Vector3 );
    void insertTemplate( QString );
    void templatePathChanged( QString );
    void insertWaypoint();

    virtual void createContextMenu( bool, int, int );
    virtual void processContextMenu( int x, int y );
    // sends back the context
    void setContext( int, std::string );

    // get the last selection ray
    void setSelectionRay( Ogre::Ray );

    void publishPointCloudWorldRequest();

    void publishMarkers();

    void resetView();

    void clearPointCloudRequests();
    void clearMapRequests();

Q_SIGNALS:
    void setRenderPanel( rviz::RenderPanel* );
    void resetSelection();
    void setMarkerScale( float );
    // send position of the mouse when clicked to create context menu
    void queryContext( int, int );
    void setMarkerPosition( float, float, float );
    void enableTemplateMarkers( bool );
    void setFrustum( const float &, const float &, const float&, const float& );
    void finishedContextMenuSetup( int x, int y );

protected:
    virtual void timerEvent(QTimerEvent *event);
    void transform(const std::string& target_frame, geometry_msgs::PoseStamped& pose);
    void transform(Ogre::Vector3& position, Ogre::Quaternion& orientation, const char* from_frame, const char* to_frame);

    void removeTemplate(int id);

    void publishGhostPoses();
    virtual rviz::ViewController* getCurrentViewController();

    void publishHandPose(std::string hand, const geometry_msgs::PoseStamped& end_effector_transform);
    void publishHandJointStates(std::string hand);
    int calcWristTarget(const geometry_msgs::PoseStamped& end_effector_pose, tf::Transform hand_T_palm, geometry_msgs::PoseStamped& final_pose);

    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;

    rviz::Display* robot_model_;
    std::vector<rviz::Display*> im_ghost_robot_;
    //std::vector<InteractiveMarkerServerCustom*> im_ghost_robot_server_;
    rviz::Display* interactive_marker_template_;
    rviz::Display* octomap_;
    rviz::Display* laser_scan_;
    rviz::Display* lidar_point_cloud_viewer_;
    rviz::Display* stereo_point_cloud_viewer_;
    rviz::Display* selection_3d_display_;
    rviz::Display* template_display_;
    rviz::Display* waypoints_display_;
    rviz::Display* achieved_waypoints_display_;
    rviz::Display* octomap_roi_;
    rviz::Display* point_cloud_request_viewer_;
    std::map<std::string,rviz::Display*> frustum_viewer_list_;

    // new displays for walking
    rviz::Display* footsteps_array_;
    std::vector<rviz::Display*> ground_map_;
    rviz::Display* goal_pose_walk_;
    rviz::Display* goal_pose_step_;
    rviz::Display* planner_start_;
    rviz::Display* planned_path_;

    rviz::Display* left_grasp_hand_model_;
    rviz::Display* right_grasp_hand_model_;

    rviz::Display* left_hand_model_;
    robot_model_loader::RobotModelLoaderPtr left_hand_model_loader_;
    robot_model::RobotModelPtr left_hand_robot_model_;
    robot_state::RobotStatePtr left_hand_robot_state_;
    moveit_msgs::DisplayRobotState left_display_state_msg_;
    ros::Publisher left_hand_robot_state_vis_pub_;
    // Used to make setting virtual joint positions (-> hand pose) easier
    sensor_msgs::JointState left_hand_virtual_link_joint_states_;

    rviz::Display* right_hand_model_;
    robot_model_loader::RobotModelLoaderPtr right_hand_model_loader_;
    robot_model::RobotModelPtr right_hand_robot_model_;
    robot_state::RobotStatePtr right_hand_robot_state_;
    moveit_msgs::DisplayRobotState right_display_state_msg_;
    ros::Publisher right_hand_robot_state_vis_pub_;
    // Used to make setting virtual joint positions (-> hand pose) easier
    sensor_msgs::JointState right_hand_virtual_link_joint_states_;

    // for simulation
    rviz::Display* ghost_robot_model_;

    std::map<std::string,rviz::Display*> display_list_;

    rviz::Tool* interactive_markers_tool_;
    //rviz::Tool* selection_tool_;
    rviz::Tool* move_camera_tool_;
    rviz::Tool* set_walk_goal_tool_;
    rviz::Tool* set_step_goal_tool_;

    Ogre::Vector3 selection_position_;

    ros::NodeHandle nh_;

    ros::Publisher template_add_pub_;
    ros::Publisher waypoint_add_pub_;

    ros::Publisher octomap_roi_pub_;

    ros::Publisher global_selection_pos_pub_;
    ros::Subscriber global_selection_pos_sub_;

    ros::Subscriber ground_map_sub_;
    ros::Subscriber point_cloud_result_sub_;

    ros::Publisher pointcloud_request_world_pub_;

    std::vector<ros::Subscriber> end_effector_sub_;
    ros::Publisher end_effector_pub_;
    ros::Publisher ghost_root_pose_pub_;
    std::map<std::string,geometry_msgs::PoseStamped> end_effector_pose_list_;

    ros::Subscriber ghost_control_state_sub_;

    ros::Publisher ghost_joint_state_pub_;
    ros::Subscriber joint_states_sub_;
    ros::Subscriber reset_pelvis_sub_;
    ros::Subscriber send_pelvis_sub_;
    ros::Publisher send_footstep_goal_pub_;

    ros::Publisher interactive_marker_add_pub_;
    ros::Publisher interactive_marker_update_pub_;
    ros::Subscriber interactive_marker_feedback_sub_;

    ros::Subscriber ghost_hand_left_sub_;
    ros::Subscriber ghost_hand_right_sub_;

    std::vector<unsigned char> ghost_planning_group_;
    std::vector<unsigned char> ghost_pose_source_;
    std::vector<unsigned char> ghost_world_lock_;
    unsigned char moveit_collision_avoidance_;
    unsigned char ghost_lock_pelvis_;
    bool update_markers_;
    bool snap_ghost_to_robot_;
    bool left_marker_moveit_loopback_;
    bool right_marker_moveit_loopback_;
    
    vigir_ocs::MouseEventHandler* mouse_event_handler_;

    std::string base_frame_;

    bool selected_;
    QString selected_template_path_;

    int active_context_;

    Ogre::Ray last_selection_ray_;

    int marker_published_;
    int stored_maps_;// THIS VALUE DETERMINES HOW MANY WE STORE

    bool moving_pelvis_;
    bool moving_l_arm_;
    bool moving_r_arm_;

    bool visualize_grid_map_;
    QWidget* position_widget_;
    QLineEdit* position_label_;

    QPushButton* reset_view_button_;

    tf::Transform l_hand_T_palm_;
    tf::Transform r_hand_T_palm_;

    QBasicTimer timer;

    int view_id_;

    std::string l_hand_type, r_hand_type;

    QMenu context_menu_;
    QAction* context_menu_selected_item_;

    int initializing_context_menu_;
    std::string active_context_name_;

    ros::Publisher template_remove_pub_;
};
}
#endif // BASE_3D_VIEW_H
