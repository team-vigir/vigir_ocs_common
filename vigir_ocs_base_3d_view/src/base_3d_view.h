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
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneManager.h>
#include <OgreSubEntity.h>
#include <OgreHighLevelGpuProgramManager.h>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <mouse_event_handler.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <vigir_interactive_marker_server_custom/interactive_marker_server_custom.h>
#include <vigir_ocs_msgs/OCSInteractiveMarkerAdd.h>
#include <vigir_ocs_msgs/OCSInteractiveMarkerUpdate.h>
#include <vigir_ocs_msgs/OCSKeyEvent.h>
#include <vigir_ocs_msgs/OCSHotkeyRelay.h>
#include <vigir_ocs_msgs/OCSObjectSelection.h>
#include <vigir_ocs_msgs/OCSCameraTransform.h>
#include <vigir_ocs_msgs/OCSControlMode.h>
#include "vigir_ocs_msgs/OCSSynchronize.h"
//#include <vigir_ocs_msgs/OCSMarkerVisibility.h>
#include <vigir_ocs_msgs/OCSGraspSync.h>
#include <vigir_perception_msgs/RaycastRequest.h>
#include <vigir_perception_msgs/PointCloudTypeRegionRequest.h>
#include <vigir_control_msgs/VigirControlModeCommand.h>
#include <vigir_control_msgs/VigirControlMode.h>

#include "robot_custom.h"
#include "robot_link_custom.h"

#include <string>
#include <boost/bind.hpp>
#include <vector>
#include <map>
#include <stdlib.h>

#include "robot_state_manager.h"
#include "notification_system.h"
#include "comms_notification_system.h"
#include "context_menu_manager.h"
#include "hotkey_manager.h"

#include <tf_conversions/tf_eigen.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

#include <vigir_planning_msgs/PlannerConfiguration.h>

// local includes
#include "footstep_vis_manager.h"


namespace rviz
{
class Display;
class Tool;
class RenderPanel;
class RenderPanelCustom;
class VisualizationManager;
class ViewController;
class FrameManager;
class OrbitViewController;
class FPSViewController;
}

//class MoveItOcsModel;

namespace vigir_ocs
{
class CircularMotionWidget;
struct CircularMotionSettings;

class CartesianMotionWidget;
struct CartesianMotionSettings;

class BaseContextMenu;

// Class "Main3DView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class Base3DView: public QWidget
{
    Q_OBJECT

    //friend to access certain protected methods for context menu callbacks
    friend class BaseContextMenu;    

public:

    /**
      * @param copy_from Another instance of Base3DView containing the main rviz instance
      * @param base_frame Defines the frame in which this window will be rendered
      * @param widget_name The name of this window
      * @param parent Parent widget of this window
      * Class "Main3DView" implements the RobotModel class with joint manipulation that can be added to any QT application.
      */
    Base3DView( Base3DView* copy_from = NULL, std::string base_frame = "/pelvis", std::string widget_name = "", QWidget *parent = 0 );
    virtual ~Base3DView();

    /**
      * ROS Callback: Receives occupancy map from onboard based on the slice of octomap
      */
    void processNewMap(const nav_msgs::OccupancyGrid::ConstPtr pose);

    /**
      * ROS Callback: Processes a new selection pose from the ui (done with ctrl click)
      */
    void processNewSelection( const geometry_msgs::Point::ConstPtr pose );

    /**
      * ROS Callback: Sets selection pose based on point cloud received from onboard
      */
    void processPointCloud( const sensor_msgs::PointCloud2::ConstPtr pc );


    BaseContextMenu * getBaseContextMenu(){return base_context_menu_;}
    ContextMenuManager * getContextMenuManager(){return context_menu_manager_;}
    std::string getActiveContext(){return active_context_name_;}
    std::vector<rviz::Display*> getCartesianMarkerList(){return cartesian_marker_list_;}
    rviz::Display* getCircularMarker(){return circular_marker_;}    
    bool getGhostLeftHandLocked(){return ghost_left_hand_lock_;}
    bool getGhostRightHandLocked(){return ghost_right_hand_lock_;}
    rviz::Tool* getInteractiveMarkersTool(){return interactive_markers_tool_;}
    //rviz::Tool* getMoveCameraTool(){return move_camera_tool_;}
    rviz::Tool* getSetGoalTool(){return set_goal_tool_;}



    /**
      * ROS Callback: receives left arm end effector position from moveit
      */
    void processLeftArmEndEffector( const geometry_msgs::PoseStamped::ConstPtr pose );
    /**
      * ROS Callback: receives right armend effector position from moveit
      */
    void processRightArmEndEffector( const geometry_msgs::PoseStamped::ConstPtr pose );
    /**
      * ROS Callback: receives pelvis end effector position from moveit
      */
    void processPelvisEndEffector( const geometry_msgs::PoseStamped::ConstPtr pose );

    /**
      * ROS Callback: receives left hand position to show grasps
      */
    void processLeftGhostHandPose( const geometry_msgs::PoseStamped::ConstPtr pose );
    /**
      * ROS Callback: receives right hand position to show grasps
      */
    void processRightGhostHandPose( const geometry_msgs::PoseStamped::ConstPtr pose );

    /**
      * ROS Callback: receives joint states from the robot
      * calculates joint effort and position limits proceeds to call updateJointIcons()
      * TODO Create helper function for calculating joint efforts
      */
    void processJointStates( const sensor_msgs::JointState::ConstPtr states );

    /**
      * ROS Callback: receives joint states from the ghost robot
      * calculates joint effort and position limits proceeds to call updateJointIcons()
      * TODO Create helper function for calculating joint efforts
      */
    void processGhostJointStates(const sensor_msgs::JointState::ConstPtr states);

    /**
      * ROS Callback: receives request to reset ghost robot pose to robot pose
      */
    void processPelvisResetRequest( const std_msgs::Bool::ConstPtr msg );

    /**
      * ROS Callback: trigger to send footstep planner request based on ghost
      */
    void processSendPelvisToFootstepRequest( const std_msgs::Bool::ConstPtr msg );

    /**
      * ROS Callback: receives the current control mode from onboard
      */
    void processControlMode( const vigir_control_msgs::VigirControlMode::ConstPtr msg );

    /**
      * ROS Callback: receives configuration message for ghost robot
      */
    void processSendCartesian( const std_msgs::Bool::ConstPtr msg );

    /**
      * ROS Callback: receives a new pose for the ghost robot
      */
    void processGhostPelvisPose(const geometry_msgs::PoseStamped::ConstPtr msg);

    /**
      * ROS Callback: receives a new selected object and enables interactive marker if possible
      */
    void processObjectSelection(const vigir_ocs_msgs::OCSObjectSelection::ConstPtr msg);

    /**
      * ROS Callback: receives hotkey from secondary OCS
      */
    virtual void processHotkeyRelayMessage(const vigir_ocs_msgs::OCSHotkeyRelay::ConstPtr msg);
    /**
      * ROS Callback: receives new goal pose for footstep planner
      */
    virtual void processOwnGoalPose(const geometry_msgs::PoseStamped::ConstPtr pose);
    virtual void processGoalPose( const geometry_msgs::PoseStamped::ConstPtr pose );
    /**
      * ROS Callback: receives new key event from global hotkey process
      */
    virtual void processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr key_event);

    void processGraspSyncCB(const vigir_ocs_msgs::OCSGraspSync::ConstPtr msg);

    void processCommsStatus(const std_msgs::Int8ConstPtr msg);

    void processPlannerConfiguration(const vigir_planning_msgs::PlannerConfiguration::ConstPtr msg);

    /**
      * ROS Callback: receives interactive marker pose updates
      */
    void onMarkerFeedback( const vigir_ocs_msgs::OCSInteractiveMarkerUpdate& msg );//std::string topic_name, geometry_msgs::PoseStamped pose);

    // functions needed for shared contexts
    rviz::VisualizationManager* getVisualizationManager() { return manager_; }
    rviz::Display* getSelection3DDisplay() { return selection_3d_display_; }
    rviz::Display* getNotificationOverlayDisplay() { return notification_overlay_display_; }    
    MouseEventHandler* getMouseEventHander() { return mouse_event_handler_; }    


    /**
      * Changes the OGRE render mask for this window which determines which object will be rendered
      */
    void updateRenderMask( bool );

    // returns name of the class that was instanced
    std::string getWidgetName() { return widget_name_; }

    /**
      * Utility method for comparing two poses with position and orientation thresholds
      */
    static bool checkPoseMatch(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, float scalar_error_threshold = 0.0f, float angle_error_threshold = 0.0f);

    // returns the footstep manager
    FootstepVisManager* getFootstepVisManager() { return footstep_vis_manager_; }
    /**
      * ROS Callback:synchronize 3d views on reset requests/toggles
      */
    void synchronizeViews(const vigir_ocs_msgs::OCSSynchronize::ConstPtr msg);

    //callbacks to receive ghost state  data
    void stateSnapGhostToRobot(const std_msgs::Bool::ConstPtr msg);
    void stateUseTorsoCB(const std_msgs::Bool::ConstPtr msg);
    void stateLockPelvisCB(const std_msgs::Bool::ConstPtr msg);
    void statePositionOnlyIkCB(const std_msgs::Bool::ConstPtr msg);
    void stateUseDrakeIkCB(const std_msgs::Bool::ConstPtr msg);

    // public hotkey callbacks - for hotkeys that shouldn't be defined for every window
    void executeStepPlanHotkey();

public Q_SLOTS:
    // displays
    // Enables/disables visibility of rviz displays
    void robotModelToggled( bool );
    void graspModelToggled( bool );
    void templatesToggled( bool );
    void requestedPointCloudToggled( bool );
    void lidarPointCloudToggled( bool );
    void stereoPointCloudToggled( bool );
    void laserScanToggled( bool );
    void ft_sensorToggled( bool );
    void markerArrayToggled( bool );
    void gridMapToggled( bool );
    void footstepPlanningToggled( bool );
    void simulationRobotToggled( bool );
    void notificationSystemToggled(bool);
    void updateGhostRobotOpacityToggled(bool);
    void cameraFrustumToggled(bool);
    // tools
    // enables/disables use of rviz tools
    void cameraToggled( bool );
    void selectToggled( bool );
    void select3DToggled( bool );
    void markerRobotToggled( bool );
    void markerTemplateToggled( bool );
    void robotJointMarkerToggled(bool selected);
    void robotOcclusionToggled(bool selected);
    virtual void defineFootstepGoal();

    /**
      * Sets position of new selection marker
      */
    void newSelection( Ogre::Vector3 );
    /**
      * adds new template with name
      */
    void insertTemplate( QString );

    /**
      * Change the system path where templates are located
      */
    void templatePathChanged( QString );
    /**
      * Insert navigation waypoint (not used anymore)
      */
    void insertWaypoint();

    /**
      * sends back the context
      */
    void setContext( int, std::string );

    void emitQueryContext(int x,int y);

    /**
      * get the last selection ray
      */
    void setSelectionRay( Ogre::Ray );
    /**
      * Requests a point cloud based on the selection point (ctrl click)
      */
    void publishPointCloudWorldRequest();

    /**
      * Creates Interactive markers for end effectors
      */
    void publishMarkers();

    /**
      * Centers view on the robot pelvis
      */
    void resetView();

    /**
      * Resets the point cloud display for ctrl click point cloud request
      */
    void clearPointCloudRaycastRequests();
    /**
      * Resets point cloud display for stereo point cloud regions
      */
    void clearPointCloudStereoRequests();
    /**
      *  Resets point cloud display for lidar point cloud regions
      */
    void clearPointCloudRegionRequests();
    /**
      * Resets area maps
      */
    void clearMapRequests();
    /**
      * Removes collsioin objects from the planning scene
      */
    void clearPlanningObjects();

    /**
      * send pose to moveit and requests cartesian plan
      */
    void sendCartesianToArm();
     /**
      * send pose and radius to moveit and requests circular plan
      */
    void sendCircularToArm();


    /**
      * Select object on double click
      */
    void selectOnDoubleClick(int,int);

    virtual bool eventFilter( QObject * o, QEvent * e );

    /**
      * Changes render order of objects in rviz
      * Render Queue Main |  PointClouds, Robot (opaque parts) ,opaque objects
      *               +1  |  Transparent Objects
      */
    //void setRenderOrder();
    //void resetRenderOrder();

    int getSelectedTemplate(){return last_selected_template_id_;}



Q_SIGNALS:
    /**
      * Sets the render panel
      */
    void setRenderPanel( rviz::RenderPanel* );
    /**
      * resets the selection
      */
    void resetSelection();
    /**
      * send position of the mouse when clicked to create context menu
      */
    void queryContext( int, int );            
    /**
      * Sets the ctr-click marker position
      */
    void setMarkerPosition( float, float, float );
    /**
      * enables/disables a template marker using the template id
      */
    void enableTemplateMarker( int, bool );
    /**
      * Enables/disables all template markers
      */
    void enableTemplateMarkers( bool );
    /**
      * emit signal to indicate that the context menu has been processed
      */
    //void finishedContextMenuSetup( int x, int y );
    /**
      * Sends the current ctr-click position as text
      */
    void sendPositionText(QString s);
    /**
      * Handler for the large red stop button
      */
    void emergencyStop();
    /**
      * send fps value to be displayed on screen
      */
    void sendFPS(int);




protected:
    virtual void timerEvent(QTimerEvent *event);

    void updateGhostRobotOpacity();
    void showAllGhost();

    /**
      * Adds joint disks that visualize the current state of the joints
      */
    void updateJointIcons(const std::string& name, const geometry_msgs::Pose& pose,double effortPercent, double boundPercent, bool ghost, int arrowDirection);
    /**
      * Lock arm to template using arm id
      */
    void setTemplateGraspLock(int arm);
    /**
      * transform pose to the target frame
      */
    void transform(const std::string& target_frame, geometry_msgs::PoseStamped& pose);
    /**
      * transform to the target frame
      */
    void transform(Ogre::Vector3& position, Ogre::Quaternion& orientation, const char* from_frame, const char* to_frame);

    virtual rviz::ViewController* getCurrentViewController();

    /**
      * publish current camera transform
      */
    void publishCameraTransform();

    bool lock_rotation_;
    bool lock_world_;
    int interactive_marker_mode_;


    Ogre::Camera* getCamera();

    NotificationSystem* notification_system_;   

    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;

    rviz::Display* robot_model_;    
    rviz::Display* interactive_marker_template_;
    rviz::Display* octomap_;
    rviz::Display* grid_;
    rviz::Display* laser_scan_;
    rviz::Display* lidar_point_cloud_viewer_;
    rviz::Display* stereo_point_cloud_viewer_;
    rviz::Display* lidar_mesh_viewer_;
    rviz::Display* stereo_mesh_viewer_;
    rviz::Display* selection_3d_display_;
    rviz::Display * notification_overlay_display_;
    rviz::Display * comms_status_overlay_display_;
    rviz::Display* template_display_;
    rviz::Display* waypoints_display_;
    rviz::Display* achieved_waypoints_display_;
    rviz::Display* octomap_roi_;
    rviz::Display* raycast_point_cloud_viewer_;
    rviz::Display* joint_arrows_;
    rviz::Display* ghost_joint_arrows_;
    rviz::Display* frustum_display_;
    std::map<std::string,rviz::Display*> frustum_map_;

    // list of gridmaps to be displayed
    std::vector<rviz::Display*> ground_map_;

    rviz::Display*  left_ft_sensor_;
    rviz::Display*  right_ft_sensor_;

    rviz::Tool*     interactive_markers_tool_;
    //rviz::Tool* selection_tool_;
    //rviz::Tool* move_camera_tool_;
    rviz::Tool*     set_goal_tool_;

    Ogre::Vector3   selection_position_;

    ros::NodeHandle nh_;

    ros::Publisher  template_add_pub_;
    ros::Publisher  waypoint_add_pub_;

    ros::Publisher  octomap_roi_pub_;

    ros::Publisher  global_selection_pos_pub_;
    ros::Subscriber global_selection_pos_sub_;

    ros::Subscriber ground_map_sub_;
    ros::Subscriber point_cloud_result_sub_;

    ros::Publisher  pointcloud_request_world_pub_;

    ros::Publisher  send_footstep_goal_pub_;
    ros::Subscriber send_footstep_goal_sub_;

    ros::Publisher  set_goal_pub_;
    ros::Subscriber set_goal_sub_;

    ros::Publisher  interactive_marker_add_pub_;
    ros::Publisher  interactive_marker_update_pub_;
    ros::Subscriber interactive_marker_feedback_sub_;
    ros::Publisher  interactive_marker_remove_pub_;
    ros::Publisher  interactive_marker_server_mode_pub_;
    ros::Subscriber interactive_marker_server_mode_sub_;

    ros::Publisher  flor_mode_command_pub_;
    ros::Subscriber flor_mode_sub_;
    ros::Subscriber robot_joint_state_sub_;

    ros::Publisher  select_object_pub_;
    ros::Subscriber select_object_sub_;

    ros::Publisher  camera_transform_pub_;
    ros::Subscriber camera_transform_sub_;

    ros::Subscriber ocs_sync_sub_;
    ros::Publisher  ocs_sync_pub_;

	ros::Publisher clear_planning_objects_pub_;

    ros::Subscriber grasp_sync_sub_;
    ros::Publisher  grasp_sync_pub_;

    ros::Publisher  interactive_marker_visibility_pub_;

    ros::Subscriber comms_status_sub_;

    ros::Publisher snap_ghost_to_robot_pub_;

    //subscribers to grab ghost state data
    ros::Subscriber state_use_torso_sub_;
    ros::Subscriber state_snap_ghost_to_robot_sub_;
    ros::Subscriber state_lock_pelvis_sub_;
    ros::Subscriber state_position_only_ik_sub_;
    ros::Subscriber state_use_drake_ik_sub_;

    bool ghost_left_hand_lock_;
    bool ghost_right_hand_lock_ ;

    bool ghost_use_torso_;

    vigir_ocs::MouseEventHandler* mouse_event_handler_;

    std::string base_frame_;
    std::string widget_name_;

    bool    selected_;
    QString selected_template_path_;

    int active_context_;

    int last_footstep_plan_type_;

    Ogre::Ray last_selection_ray_;

    int stored_maps_;// THIS VALUE DETERMINES HOW MANY WE STORE

    bool       visualize_grid_map_;
    QWidget*   position_widget_;
    QLineEdit* position_label_;

    QPushButton* reset_view_button_;
    QPushButton* stop_button_;

    QBasicTimer timer_;

    int  ghost_opacity_update_counter_;
    int  ghost_opacity_update_frequency_;
    bool ghost_opacity_update_;

    int  view_id_;

    ros::Publisher template_remove_pub_;

    int flor_atlas_current_mode_;

    std::vector<std::string> keys_pressed_list_;

    ros::Subscriber key_event_sub_;
    ros::Subscriber hotkey_relay_sub_;

    bool is_primary_view_;

    int last_selected_template_id_;




    ////////////////////
    // occluded robot

    std::map<std::string,rviz::Display*> jointDisplayMap;

    typedef std::map<std::string, rviz::RobotLinkCustom*> M_NameToLink;
    typedef std::map<Ogre::SubEntity*, Ogre::MaterialPtr> M_SubEntityToMaterial;
    /**
      * Enables reordering of renderque
      */
    //void setRobotOccludedRender();
    /**
      * Disables rendering of robot occluded
      */
    //void disableRobotOccludedRender();
    /**
      * Helper function for reordering the render queue
      */
    //void setSceneNodeRenderGroup(Ogre::SceneNode* sceneNode, int queueOffset);

    bool disable_joint_markers_;
    //flag to disable extra calls to setting render order in timer function
    bool occluded_robot_visible_;



    ////////////////////
    // selection

    /**
      * Helper Function: deselects all objects in the current view
      **/
    void deselectAll();
    /**
      * Select a template
      */
    void selectTemplate(int id=-1);

    /**
      * Select a footstep
      */
    void selectFootstep();
    /**
      * Select a footstep from the step plan goal
      */
    void selectFootstepGoal();
    /**
      * select left arm end effector
      */
    void selectLeftArm();
    /**
      * select right arm end effector
      */
    void selectRightArm();
    /**
      * select pelvis marker
      */
    void selectPelvis();
    /**
      *  select the starting footstep and send to footstep vis manager
      */
    void setStartingFootstep();

    void lockFootstep();

    void unlockFootstep();

    void removeFootstep();


    ////////////////////
    // context menu
    /**
      * Context menu action for inserting a template
      */
    void insertTemplateContextMenu();
    /**
      * Context menu action for removing a template
      */
    void removeTemplateContextMenu();
    /**
      * Context menu action for executing a footstep plan
      */

    BaseContextMenu * base_context_menu_;    
    ContextMenuManager* context_menu_manager_;

    std::string active_context_name_;

    /**
      * removes template using id
      */
    void removeTemplate(int id);
    /**
      * Gives object context on right click
      */
    int findObjectContext(std::string obj_type);


    ////////Hot key/////////////////////
    void addHotkeys();
    //Callbacks
    void resetEverythingHotkey();
    void showEStopHotkey();
    void resetPointCloudsHotkey();
    void rainbowColorHotkey();
    void pointcloudIntensityHotkey();
    //void requestStepPlanHotkey();
    void lockWorldHotkey();
    void lockTranslationHotkey();
    void snapGhostHotkey();
    void stereoMeshHotkey();
    void lidarMeshHotkey();


    ////////////////////
    // Cartesian/circular moveit
    /**
      * Context menu action for creating a cartesian target point
      */
    void createCartesianContextMenu();
    /**
      * Context menu action for creating a cartesian target point from ghost robot's left 
      */
    void createCartesianMarkerFromGhostLeftHand();
    /**
      * Context menu action for creating a cartesian target point from ghost robot's right
      */
    void createCartesianMarkerFromGhostRightHand();
    /**
      * Create cartesian motion marker at a given pose
      */
    void createCartesianMarkerForPose(geometry_msgs::Pose &pose);
    /**
      * Context menu action for removing a cartesian target point
      */
    void removeCartesianContextMenu();
    /**
      * Context menu action for creating a circular target point
      */
    void createCircularContextMenu();
    /**
      * Context menu action for removing a circular target point
      */
    void removeCircularContextMenu();
    /**
      * Publishes the cartesial target
      */
    void sendCartesianTarget(bool right_hand, std::vector<geometry_msgs::Pose> waypoints, vigir_ocs::CartesianMotionSettings &motion_settings);
    /**
      * Publishes the circular target pose
      */
    void sendCircularTarget(bool right_hand, vigir_ocs::CircularMotionSettings &motion_settings);

    std::vector<rviz::Display*> cartesian_marker_list_;
    rviz::Display* circular_marker_;

    std::vector<geometry_msgs::Pose> cartesian_waypoint_list_;
    geometry_msgs::Pose circular_center_;

    ros::Publisher cartesian_plan_request_pub_;
    ros::Publisher circular_plan_request_pub_;

    CartesianMotionWidget* cartesian_config_widget_;
    CircularMotionWidget* circular_config_widget_;

    ros::Subscriber send_cartesian_sub_;

    ////////////////////
    // ghost

    rviz::Display* ghost_robot_model_;

    sensor_msgs::JointState::ConstPtr latest_ghost_joint_state_;

    geometry_msgs::Pose last_l_arm_moveit_pose_;
    geometry_msgs::Pose last_r_arm_moveit_pose_;
    geometry_msgs::Pose last_l_arm_marker_pose_;
    geometry_msgs::Pose last_r_arm_marker_pose_;
    bool update_l_arm_color_;
    bool update_r_arm_color_;

    ros::Subscriber send_ghost_pelvis_pose_sub_;
    ros::Subscriber ghost_joint_state_sub_;
    geometry_msgs::Pose ghost_root_pose_;

    ros::Publisher l_arm_marker_pose_pub_;
    ros::Publisher r_arm_marker_pose_pub_;
    ros::Publisher pelvis_marker_pose_pub_;

    vigir_ocs_msgs::OCSInteractiveMarkerUpdate l_arm_marker_update;
    vigir_ocs_msgs::OCSInteractiveMarkerUpdate r_arm_marker_update;

    int marker_published_;

    bool moving_pelvis_;
    bool moving_l_arm_;
    bool moving_r_arm_;

    std::vector<unsigned char> ghost_planning_group_;
    bool ghost_lock_pelvis_;
    bool update_markers_;
    bool snap_ghost_to_robot_;
    bool snap_left_hand_to_ghost_;
    bool snap_right_hand_to_ghost_;
    bool left_marker_moveit_loopback_;
    bool right_marker_moveit_loopback_;
    bool position_only_ik_;

    vigir_planning_msgs::PlannerConfiguration planner_configuration_;
    ros::Publisher planner_configuration_pub_;
    ros::Subscriber planner_configuration_sub_;


    std::vector<ros::Subscriber> end_effector_sub_;
    ros::Publisher end_effector_pub_;
    ros::Publisher ghost_root_pose_pub_;
    std::map<std::string,geometry_msgs::PoseStamped> end_effector_pose_list_;

    ros::Subscriber ghost_control_state_sub_;

    ros::Publisher ghost_joint_state_pub_;
    ros::Subscriber joint_states_sub_;
    ros::Subscriber reset_pelvis_sub_;
    ros::Subscriber send_pelvis_sub_;

    ros::Subscriber ghost_hand_left_sub_;
    ros::Subscriber ghost_hand_right_sub_;

    tf::Transform l_hand_T_palm_;
    tf::Transform r_hand_T_palm_;

    tf::Transform l_hand_T_marker_;
    tf::Transform r_hand_T_marker_;

    std::string left_wrist_link_, right_wrist_link_;
    std::string left_palm_link_,  right_palm_link_;
    std::string left_hand_group_, right_hand_group_;

    std::vector<rviz::Display*> im_ghost_robot_;

    rviz::Display* left_grasp_hand_model_;
    rviz::Display* right_grasp_hand_model_;

    rviz::Display* left_hand_model_;
    robot_model_loader::RobotModelLoaderPtr left_hand_model_loader_;
    robot_model::RobotModelPtr left_hand_robot_model_;
    robot_state::RobotStatePtr left_hand_robot_state_;
    std::vector<std::string>   left_hand_joint_names_;
    moveit_msgs::DisplayRobotState left_display_state_msg_;
    ros::Publisher left_hand_robot_state_vis_pub_;
    // Used to make setting virtual joint positions (-> hand pose) easier
    sensor_msgs::JointState left_hand_virtual_link_joint_states_;

    rviz::Display* right_hand_model_;
    robot_model_loader::RobotModelLoaderPtr right_hand_model_loader_;
    robot_model::RobotModelPtr right_hand_robot_model_;
    robot_state::RobotStatePtr right_hand_robot_state_;
    std::vector<std::string>   right_hand_joint_names_;
    moveit_msgs::DisplayRobotState right_display_state_msg_;
    ros::Publisher right_hand_robot_state_vis_pub_;
    // Used to make setting virtual joint positions (-> hand pose) easier
    sensor_msgs::JointState right_hand_virtual_link_joint_states_;

    //Whole robot model
    robot_model_loader::RobotModelLoaderPtr    robot_urdf_model_loader_;
    robot_model::RobotModelPtr                 robot_urdf_model_;

    rviz::Display* left_hand_bounding_box_;
    rviz::Display* right_hand_bounding_box_;
    rviz::Display* pelvis_hand_bounding_box_;

    moveit_msgs::DisplayRobotState ghost_display_state_msg_;
    ros::Publisher ghost_robot_state_vis_pub_;        

    /**
      * Callback for setting im mode
      */
    void processInteractiveMarkerMode(const vigir_ocs_msgs::OCSControlMode::ConstPtr msg);

    /**
      * Updates color of the hands based on moveit
      */
    void updateHandColors();

    /**
      * Processes ghost joint states. Used for updating joint icons
      */
    void processGhostJoints(const std::string& name, const geometry_msgs::Pose& pose);

    /**
      * published the pose of the IM of the ghost robot
      */
    void publishGhostPoses(bool local_feedback = false);

    /**
      * Sets end effector pose
      */
    void publishHandPose(std::string hand, const geometry_msgs::PoseStamped& end_effector_transform);
    /**
      * published hand/finger joint states
      */
    void publishHandJointStates(std::string hand);
    /**
      * Transforms end effector position to wrist position
      */
    int calcWristTarget(const geometry_msgs::PoseStamped& end_effector_pose, tf::Transform hand_T_palm, geometry_msgs::PoseStamped& final_pose);
    /**
      * Snaps hand to current ghost position
      */
    void snapHandGhost();

    void initializeFrustums(std::string prefix);

    void blurRender();
    void setChildrenVisibility(Ogre::SceneNode* node, std::vector<bool>& last_visibility, bool visibility);
    void restoreChildrenVisibility(Ogre::SceneNode* node, std::vector<bool>& last_visibility);

    Ogre::RenderTexture *renderTexture1;
    Ogre::RenderTexture *renderTexture2;
    Ogre::RenderTexture *renderTexture3;
    ///////////////////
    // new managers
    FootstepVisManager* footstep_vis_manager_;

    bool use_drake_ik_;
};
}
#endif // BASE_3D_VIEW_H
