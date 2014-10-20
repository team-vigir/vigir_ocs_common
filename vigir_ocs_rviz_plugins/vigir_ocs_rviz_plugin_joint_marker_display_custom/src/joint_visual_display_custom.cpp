
#include "joint_visual_display_custom.h"


/*
  Added ability to change colors. Scale/position/color are no longer based on effort

  */
namespace rviz
{

    JointVisualCustom::JointVisualCustom( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, boost::shared_ptr<urdf::Model> urdf_model )
    {
        scene_manager_ = scene_manager;

        // Ogre::SceneNode s form a tree, with each node storing the
        // transform (position and orientation) of itself relative to its
        // parent.  Ogre does the math of combining those transforms when it
        // is time to render.
        //
        // Here we create a node to store the pose of the Effort's header frame
        // relative to the RViz fixed frame.
        frame_node_ = parent_node->createChildSceneNode();

        //
        urdf_model_ = urdf_model;

        // We create the arrow object within the frame node so that we can
        // set its position and direction relative to its header frame.
        for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it = urdf_model_->joints_.begin(); it != urdf_model_->joints_.end(); it ++ )
        {
            if ( it->second->type == urdf::Joint::REVOLUTE )
            {
                std::string joint_name = it->first;
            }
        }
        //default to green
        color_ = new Ogre::ColourValue(0,1.0,0,.9f);

        marker_scale_ = .08f;
        //change this to make animation faster/slower
        arrow_step_interval_ = .10f;
        arrow_timer_ = arrow_step_interval_;
    }

    JointVisualCustom::~JointVisualCustom()
    {
        // Delete the arrow to make it disappear.
        for (std::map<std::string, rviz::BillboardLine*>::iterator it = effort_circle_.begin(); it != effort_circle_.end(); it ++ )
        {
            delete (it->second);
        }
        for (std::map<std::string, rviz::Arrow*>::iterator it = effort_arrow_.begin(); it != effort_arrow_.end(); it ++ )
        {
            delete (it->second);
        }

        // Destroy the frame node since we don't need it anymore.
        scene_manager_->destroySceneNode( frame_node_ );

        delete(color_);
    }

    void JointVisualCustom::setMessage( const sensor_msgs::JointState::ConstPtr& msg )
    {
       joint_msg_ = msg;
    }

    void JointVisualCustom::update( float wall_dt, float ros_dt )
    {
        arrow_timer_ -= wall_dt;
        if(arrow_timer_ < 0 && joint_msg_)
        {
            // for all joints...
            int joint_num = joint_msg_->name.size();
            for (int i = 0; i < joint_num; i++ )
            {
                std::string joint_name = joint_msg_->name[i];
                const urdf::Joint* joint = urdf_model_->getJoint(joint_name).get();
                int joint_type = joint->type;
                if ( joint_type == urdf::Joint::REVOLUTE )
                {
                    if(effort_circle_.find(joint_name) == effort_circle_.end())
                    {
                        effort_circle_[joint_name] = new rviz::BillboardLine( scene_manager_, frame_node_ );
                        setRenderOrder(effort_circle_[joint_name]->getSceneNode());
                        effort_arrow_[joint_name] = new rviz::Arrow( scene_manager_, frame_node_ );
                        setRenderOrder(effort_arrow_[joint_name]->getSceneNode());
                        current_arrow_point_[joint_name] = 0;

                        setJointColor(0,1,0,joint_name);
                    }

                    //set arrow dimensions
                    effort_arrow_[joint_name]->set(0, width_*2, width_*2*1.0, width_*2*2.0);

                    //initialize circle
                    effort_circle_[joint_name]->clear();
                    effort_circle_[joint_name]->setLineWidth(width_);

                    //store next arrow index
                    int next_index;
                    //circle is made up of points.. for each point
                    for (int j = 0; j < 32; j++)
                    {
                        //calculate current point to be drawn
                        Ogre::Vector3 point = Ogre::Vector3((0.05+marker_scale_*scale_*0.5)*sin(j*2*M_PI/32), (0.05+marker_scale_*scale_*0.5)*cos(j*2*M_PI/32), 0);

                        //at skipped point?  and have a desired direction for an arrow?
                        if(j == current_arrow_point_[joint_name] && arrow_directions_.find(joint_name) != arrow_directions_.end())
                        {
                            //move arrow along circle by setting position at different points
                            effort_arrow_[joint_name]->setPosition(orientation_[joint_name] * point + position_[joint_name]);
                            //calculate next index for arrow direction
                            next_index = current_arrow_point_[joint_name] + arrow_directions_[joint_name];
                            if(next_index == -1)
                                next_index = 31;
                            else if (next_index == 32)
                                next_index = 0;
                            //arrow points to previous point or next point in the circle
                            Ogre::Vector3 next_point = Ogre::Vector3((0.05+marker_scale_*scale_*0.5)*sin(next_index*2*M_PI/32), (0.05+marker_scale_*scale_*0.5)*cos(next_index*2*M_PI/32), 0);
                            effort_arrow_[joint_name]->setDirection(orientation_[joint_name] * (next_point-point));
                        }
                        //draw point
                        effort_circle_[joint_name]->addPoint(orientation_[joint_name] * point + position_[joint_name]);
                    }

                    // set the arrow index
                    current_arrow_point_[joint_name] = next_index;

                    // add the first point again to close the circle
                    Ogre::Vector3 point = Ogre::Vector3((0.05+marker_scale_*scale_*0.5)*sin(0*2*M_PI/32), (0.05+marker_scale_*scale_*0.5)*cos(0*2*M_PI/32), 0);
                    effort_circle_[joint_name]->addPoint(orientation_[joint_name] * point + position_[joint_name]);
                }
            }
            //reset timer
            arrow_timer_ = arrow_step_interval_;
        }
    }

    void JointVisualCustom::setArrowDirection(std::string jointName,int direction)
    {        
        arrow_directions_[jointName] = direction;
    }

    void JointVisualCustom::setRenderOrder(Ogre::SceneNode* sceneNode)
    {
        for(int i =0;i<sceneNode->numAttachedObjects();i++)
        {
            Ogre::MovableObject* obj =  sceneNode->getAttachedObject(i);
            obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN + 1);
        }
        //recurse for all potential children
        for(int i =0;i<sceneNode->numChildren();i++)
        {
            setRenderOrder((Ogre::SceneNode*)sceneNode->getChild(i));
        }
    }

    // Position and orientation are passed through to the SceneNode.
    void JointVisualCustom::setFramePosition( const Ogre::Vector3& position )
    {
    frame_node_->setPosition( position );
    }

    void JointVisualCustom::setFrameOrientation( const Ogre::Quaternion& orientation )
    {
    frame_node_->setOrientation( orientation );
    }
    // Position and orientation are passed through to the SceneNode.
    void JointVisualCustom::setFramePosition( const std::string joint_name, const Ogre::Vector3& position )
    {
    position_[joint_name] = position;
    }

    void JointVisualCustom::setFrameOrientation( const std::string joint_name, const Ogre::Quaternion& orientation )
    {
    orientation_[joint_name] = orientation;
    }   

    void JointVisualCustom::setWidth( float w )
    {
        width_ = w;
    }

    void JointVisualCustom::setScale( float s )
    {
        scale_ = s;
    }

    void JointVisualCustom::setJointAlpha(float a, std::string joint_name)
    {        
        color_->a = a;
        if(effort_circle_.count(joint_name)> 0)// && effort_arrow_.count(joint_name)> 0)
        {            
            effort_circle_[joint_name]->setColor(color_->r,color_->g,color_->b,color_->a);
            effort_arrow_[joint_name]->setColor(color_->r,color_->g,color_->b,color_->a);
        }
        else
        {
            //may occur on startup with ghost
            ROS_ERROR("Tried to change non-existing joint alpha : %s\n",joint_name.c_str());
        }
    }

    void JointVisualCustom::setJointColor( float r, float g, float b, std::string joint_name)
    {
        color_->r = r;
        color_->g = g;
        color_->b = b;
        if(effort_circle_.count(joint_name)> 0)// && effort_arrow_.count(joint_name)> 0)
        {          
            effort_circle_[joint_name]->setColor(color_->r,color_->g,color_->b,color_->a);
            effort_arrow_[joint_name]->setColor(color_->r,color_->g,color_->b,color_->a);
        }
        else
        {
            ROS_ERROR("Tried to change non-existing joint color: %s \n",joint_name.c_str());
        }
    }

} // end namespace rviz

