
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
        currentArrowPoint = 0;
    }

    JointVisualCustom::~JointVisualCustom()
    {
        // Delete the arrow to make it disappear.
        for (std::map<std::string, rviz::BillboardLine*>::iterator it = effort_circle_.begin(); it != effort_circle_.end(); it ++ ) {
                delete (it->second);
            }
        for (std::map<std::string, rviz::Arrow*>::iterator it = effort_arrow_.begin(); it != effort_arrow_.end(); it ++ ) {
                delete (it->second);
            }        

        // Destroy the frame node since we don't need it anymore.
        scene_manager_->destroySceneNode( frame_node_ );

        delete(color_);
    }

    void JointVisualCustom::setMessage( const sensor_msgs::JointStateConstPtr& msg )
    {
        // for all joints...
        int joint_num = msg->name.size();
        for (int i = 0; i < joint_num; i++ )
        {
            std::string joint_name = msg->name[i];            
            const urdf::Joint* joint = urdf_model_->getJoint(joint_name).get();
            int joint_type = joint->type;
            if ( joint_type == urdf::Joint::REVOLUTE )
            {
                // enable or disable draw
                if ( effort_circle_.find(joint_name) != effort_circle_.end())
                {
                    delete(effort_circle_[joint_name]);
                    delete(effort_arrow_[joint_name]);
                    effort_circle_.erase(joint_name);
                    effort_arrow_.erase(joint_name);
                }
                if ( effort_circle_.find(joint_name) == effort_circle_.end())
                {
                    effort_circle_[joint_name] = new rviz::BillboardLine( scene_manager_, frame_node_ );
                   // setRenderOrder(effort_circle_[joint_name]->getSceneNode());
                    effort_arrow_[joint_name] = new rviz::Arrow( scene_manager_, frame_node_ );
                   // setRenderOrder(effort_arrow_[joint_name]->getSceneNode());
                }


                //tf::Transform offset = poseFromJoint(joint);
                boost::shared_ptr<urdf::JointLimits> limit = joint->limits;


                //set arrow dimensions
                effort_arrow_[joint_name]->set(0, width_*2, width_*2*1.0, width_*2*2.0);

                //arrow direction
                //arrow is placed in different location on ring based on direction                
//                if ( arrow_directions_.find(joint_name) != arrow_directions_.end() )
//                {                                                                                       //-1 or 1 for direction
//                    effort_arrow_[joint_name]->setDirection(orientation_[joint_name] * Ogre::Vector3(arrow_directions_[joint_name],0,0));
//                    //arrowPlaceOffset = arrow_directions_[joint_name];
//                }
//                //arrow position on circle
//                effort_arrow_[joint_name]->setPosition(orientation_[joint_name] * Ogre::Vector3(0, 0.05+marker_scale_*scale_*0.5, 0) + position_[joint_name]);

                effort_circle_[joint_name]->clear();
                effort_circle_[joint_name]->setLineWidth(width_);

                //circle is made up of points.. for each point
                for (int i = 0; i < 32; i++)
                {
                    //calculate current point to be drawn
                    Ogre::Vector3 point = Ogre::Vector3((0.05+marker_scale_*scale_*0.5)*sin(i*2*M_PI/32), (0.05+marker_scale_*scale_*0.5)*cos(i*2*M_PI/32), 0);
                   // ROS_ERROR("point %f %f %f ",point.x,point.y,point.z);

//                    //at skipped point?  and have a desired direction for an arrow?
//                    if(i == currentArrowPoint && arrow_directions_.find(joint_name) != arrow_directions_.end())
//                    {
//                        //move arrow along circle by setting position at different points
//                        effort_arrow_[joint_name]->setPosition(orientation_[joint_name] * point + position_[joint_name]);
//                        //point 33 == point 1,fyi
//                        int next_index = i + arrow_directions_[joint_name];
//                        //arrow points to previous point or next point in the circle
//                        Ogre::Vector3 next_point = Ogre::Vector3((0.05+marker_scale_*scale_*0.5)*sin(next_index*2*M_PI/32), (0.05+marker_scale_*scale_*0.5)*cos(next_index*2*M_PI/32), 0);
//                        //ROS_ERROR("next point %f %f %f ",next_point.x,next_point.y,next_point.z);
//                        effort_arrow_[joint_name]->setDirection(next_point-point);
//                        currentArrowPoint += arrow_directions_[joint_name];
//                        //currentArrowPoint must be circular as member will cycle through this function repeatedly
//                        if(currentArrowPoint == -1)
//                            currentArrowPoint = 31;
//                        else if (currentArrowPoint == 32)
//                            currentArrowPoint = 0;
//                    }

                    //ROS_ERROR("arrow point: %d",currentArrowPoint);
                    //draw point
                    effort_circle_[joint_name]->addPoint(orientation_[joint_name] * point + position_[joint_name]);


                }


                //rotate joint position values to rotate visualization? //removes the ability to recognize correct joint orientations here, but thats desired
                //cant as joint data is constantly being updated
                //                Ogre::Quaternion jointRotation =  orientation_[joint_name];
                //                //if(joint_name.find("neck") != std::string::npos)
                //                    ROS_ERROR("BEFOREjoint: %s, %f %f %f %f",joint_name.c_str(),jointRotation.w,jointRotation.x,jointRotation.y,jointRotation.z);
                //                Ogre::Quaternion* rApplied = new Ogre::Quaternion(1.0f,0.5f,0.05f,0.05f);
                //                jointRotation = jointRotation * *rApplied;
                //                orientation_[joint_name] = jointRotation;
                //                ROS_ERROR("AFTERjoint: %s, %f %f %f %f",joint_name.c_str(),jointRotation.w,jointRotation.x,jointRotation.y,jointRotation.z);

                //                Ogre::Quaternion rotation = effort_circle_[joint_name]->getOrientation();
                //               // ROS_ERROR("before %f %f %f %f",rotation.w,rotation.x,rotation.y,rotation.z);
                //                Ogre::Quaternion* rApplied = new Ogre::Quaternion(1.0f,0.5f,0,0);
                //                rotation = rotation * *rApplied;
                //               // ROS_ERROR("after %f %f %f %f",rotation.w,rotation.x,rotation.y,rotation.z);
                //                effort_circle_[joint_name]->setOrientation(orientation_[joint_name] * rotation);
                //                delete(rApplied);

            }
        }
    }

    void JointVisualCustom::setArrowDirection(std::string jointName,int direction)
    {
        //ROS_ERROR("set arrow %s %d",jointName.c_str(),direction);
        arrow_directions_[jointName] = direction;
    }

    void JointVisualCustom::setRenderOrder(Ogre::SceneNode* sceneNode)
    {
        for(int i =0;i<sceneNode->numAttachedObjects();i++)
        {
            Ogre::MovableObject* obj =  sceneNode->getAttachedObject(i);
            obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN + 50);
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

    //all joints
    void JointVisualCustom::setAlpha(float a)
    {
        color_->a = a;
        for (std::map<std::string, rviz::BillboardLine*>::iterator it=effort_circle_.begin(); it!=effort_circle_.end(); ++it)
        {
            rviz::BillboardLine* line = it->second;
            line->setColor(color_->r,color_->g,color_->b,color_->a);
        }
        for (std::map<std::string, rviz::Arrow*>::iterator it=effort_arrow_.begin(); it!=effort_arrow_.end(); ++it)
        {
            rviz::Arrow* arrow = it->second;
            arrow->setColor(color_->r,color_->g,color_->b,color_->a);
        }
    }
    void JointVisualCustom::setJointAlpha(float a, std::string joint_name)
    {        
        color_->a = a;
        if(effort_circle_.count(joint_name)> 0)// && effort_arrow_.count(joint_name)> 0)
        {
            ROS_ERROR("set alpha %s %f", joint_name.c_str(), a);
            effort_circle_[joint_name]->setColor(color_->r,color_->g,color_->b,color_->a);
            effort_arrow_[joint_name]->setColor(color_->r,color_->g,color_->b,color_->a);
        }
        else
        {
            ROS_ERROR("Tried to change non-existing joint alpha : %s\n",joint_name.c_str());
        }
    }
    void JointVisualCustom::setColor( float r, float g, float b)
    {
        color_->r = r;
        color_->g = g;
        color_->b = b;
        for (std::map<std::string, rviz::BillboardLine*>::iterator it=effort_circle_.begin(); it!=effort_circle_.end(); ++it)
        {
            rviz::BillboardLine* line = it->second;
            line->setColor(color_->r,color_->g,color_->b,color_->a);
        }
        for (std::map<std::string, rviz::Arrow*>::iterator it=effort_arrow_.begin(); it!=effort_arrow_.end(); ++it)
        {
            rviz::Arrow* arrow = it->second;
            arrow->setColor(color_->r,color_->g,color_->b,color_->a);
        }
    }
    void JointVisualCustom::setJointColor( float r, float g, float b, std::string joint_name)
    {
        color_->r = r;
        color_->g = g;
        color_->b = b;
        if(effort_circle_.count(joint_name)> 0)// && effort_arrow_.count(joint_name)> 0)
        {
          /// ROS_ERROR("set color %s %f %f %f", joint_name.c_str(),color_->r,color_->g,color_->b);
            effort_circle_[joint_name]->setColor(color_->r,color_->g,color_->b,color_->a);
            effort_arrow_[joint_name]->setColor(color_->r,color_->g,color_->b,color_->a);
        }
        else
        {
            ROS_ERROR("Tried to change non-existing joint color: %s \n",joint_name.c_str());
        }
    }

} // end namespace rviz

