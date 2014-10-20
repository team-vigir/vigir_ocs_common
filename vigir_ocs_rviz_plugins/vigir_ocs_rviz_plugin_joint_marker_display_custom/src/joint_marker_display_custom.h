#ifndef JOINT_MARKER_DISPLAY_CUSTOM_H
#define JOINT_MARKER_DISPLAY_CUSTOM_H

#include "rviz/display.h"
#include "rviz/frame_manager.h"

#include <joint_visual_display_custom.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_listener.h>

#include <OGRE/OgreVector3.h>
#include "OGRE/OgreRoot.h"
#include "OGRE/OgreRenderSystem.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreWindowEventUtilities.h"
#include "OGRE/OgreManualObject.h"
#include "OGRE/OgreEntity.h"
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreRenderQueueListener.h>

#include <map>

#include "robot_state_manager.h"

namespace Ogre
{
class Entity;
class SceneNode;
class ManualObject;
}

namespace rviz
{
class Axes;
class RenderPanel;
class FloatProperty;
class RosTopicProperty;
class ColorProperty;
class VectorProperty;
class StringProperty;
class QuaternionProperty;
}

namespace rviz
{

/**
 * \class JointMarkerDisplayCustom
 * \brief Uses a pose from topic + offset to render a bounding object with shape, size and color
 */
class JointMarkerDisplayCustom: public Display,  public Ogre::RenderTargetListener, public Ogre::RenderQueueListener
{
Q_OBJECT
public:
  JointMarkerDisplayCustom();
  virtual ~JointMarkerDisplayCustom();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update( float wall_dt, float ros_dt );
  virtual void fixedFrameChanged();
  virtual void reset();

  void clear();

  void setArrowDirection(std::string joint_name,int direction);
  void setJointColor(QColor color,std::string joint_name);
  void setJointAlpha(float alpha,std::string joint_name);

  virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
  virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
  virtual void renderQueueStarted(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation);
  virtual void renderQueueEnded(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation);

private Q_SLOTS:
  void updateWidthAndScale();
  void updateTopic();
  void setRenderPanel(rviz::RenderPanel*);
  void updateName();
  void updateRobotDescription();
  void updateGhost();

protected:
  void setPose();
  virtual void load();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  virtual void subscribe();
  virtual void unsubscribe();

  void calculateGhostTransform(std::string link_name, Ogre::Vector3& position, Ogre::Quaternion& orientation);
  void processMessage( const sensor_msgs::JointState::ConstPtr& msg );
  void processPelvisEndEffector(const geometry_msgs::PoseStamped::ConstPtr &pose);
  // The object for urdf model
  boost::shared_ptr<urdf::Model> robot_model_;

  //
  std::string robot_description_;

  RosTopicProperty* topic_property_;
  FloatProperty * width_property_,* scale_property_;
  StringProperty* name_property_;
  StringProperty *robot_description_property_;
  BoolProperty * is_ghost_property_;


  ros::NodeHandle nh_;

  ros::Subscriber joint_states_sub_;

  boost::shared_ptr<JointVisualCustom> visual_;

  std::vector<RenderPanel*> render_panel_list_;
  RenderPanel* render_panel_; // this is the active render panel

  ros::Subscriber ghost_root_sub_;
  geometry_msgs::Pose ghost_root_pose_;

  bool initialized_;
  bool is_ghost_;
};

} // namespace rviz

#endif // JOINT_MARKER_DISPLAY_CUSTOM_H
