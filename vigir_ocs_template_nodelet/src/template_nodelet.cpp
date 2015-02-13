#include "template_nodelet.h"

namespace ocs_template
{
void TemplateNodelet::onInit()
{
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle nh_out(nh, "template");
    ros::NodeHandle nhp("~");

    // also create a publisher to set parameters of cropped image
    template_list_pub_         = nh_out.advertise<flor_ocs_msgs::OCSTemplateList>( "list", 1, false );
    grasp_selected_pub_        = nh_out.advertise<flor_grasp_msgs::GraspSelection>( "grasp_selected", 1, false );
    grasp_selected_state_pub_  = nh_out.advertise<flor_grasp_msgs::GraspState>( "grasp_selected_state", 1, false );

    // then, subscribe to the resulting cropped image
    template_add_sub_            = nh_out.subscribe<flor_ocs_msgs::OCSTemplateAdd>( "add", 1, &TemplateNodelet::addTemplateCb, this );
    template_remove_sub_         = nh_out.subscribe<flor_ocs_msgs::OCSTemplateRemove>( "remove", 1, &TemplateNodelet::removeTemplateCb, this );
    template_update_sub_         = nh_out.subscribe<flor_ocs_msgs::OCSTemplateUpdate>( "update", 1, &TemplateNodelet::updateTemplateCb, this );
    template_match_feedback_sub_ = nh_out.subscribe<flor_grasp_msgs::TemplateSelection>( "template_match_feedback", 1, &TemplateNodelet::templateMatchFeedbackCb, this );
    grasp_request_sub_           = nh_out.subscribe<flor_grasp_msgs::GraspSelection>( "grasp_request", 1, &TemplateNodelet::graspRequestCb, this );
    grasp_state_feedback_sub_    = nh_out.subscribe<flor_grasp_msgs::GraspState>( "grasp_state_feedback", 1, &TemplateNodelet::graspStateFeedbackCb, this );

    template_info_server_        = nh_out.advertiseService("/template_info", &TemplateNodelet::templateInfoSrv, this);

    grasp_info_server_           = nh_out.advertiseService("/grasp_info", &TemplateNodelet::graspInfoSrv, this);

    attach_object_server_        = nh_out.advertiseService("/attach_object_template", &TemplateNodelet::attachObjectTemplateSrv, this);
    stitch_object_server_        = nh_out.advertiseService("/stitch_object_template", &TemplateNodelet::stitchObjectTemplateSrv, this);
    detach_object_server_        = nh_out.advertiseService("/detach_object_template", &TemplateNodelet::detachObjectTemplateSrv, this);

    co_pub_                      = nh_out.advertise<moveit_msgs::CollisionObject>("/collision_object", 1, false);
    aco_pub_                     = nh_out.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 1, false);

    ROS_INFO(" Start reading database files");

    // which file are we reading
    if (!nhp.getParam("/r_hand_library", this->r_grasps_filename_))
    {
        ROS_ERROR(" Did not find Right Grasp Library parameter /r_hand_library");
    }
    if (!nhp.getParam("/l_hand_library", this->l_grasps_filename_))
    {
        ROS_ERROR(" Did not find Left Grasp Library parameter /l_hand_library");
    }
    if (!nhp.hasParam("ot_filename"))
    {
        ROS_ERROR(" Did not find Object Template FILENAME parameter - using \"/vigir_template_library/object_templates/object_templates.csv\" as default");
    }
    if (!nhp.hasParam("stand_filename"))
    {
        ROS_ERROR(" Did not find Ghost FILENAME parameter - using \"/vigir_template_library/robot_poses/atlas_v1_v3_joints_stand_poses.csv\" as default");
    }
    nhp.param<std::string>("ot_filename",       this->ot_filename_,        ros::package::getPath("vigir_template_library")+"/object_templates/object_templates.csv");
    nhp.param<std::string>("stand_filename",    this->stand_filename_,     ros::package::getPath("vigir_template_library")+"/robot_poses/atlas_v1_v3_joints_stand_poses.csv");

    XmlRpc::XmlRpcValue   gp_T_hand;

    // Load the Object Template database
    TemplateNodelet::loadObjectTemplateDatabase(this->ot_filename_);

    ROS_INFO("OT Database loaded");

    // Load the right hand specific grasping database
    nh.getParam("/r_hand_tf/gp_T_hand", gp_T_hand);
    gp_T_hand_.setOrigin(tf::Vector3(static_cast<double>(gp_T_hand[0]),static_cast<double>(gp_T_hand[1]),static_cast<double>(gp_T_hand[2])));
    gp_T_hand_.setRotation(tf::Quaternion(static_cast<double>(gp_T_hand[3]),static_cast<double>(gp_T_hand[4]),static_cast<double>(gp_T_hand[5]),static_cast<double>(gp_T_hand[6])));

    ROS_INFO("Right Graspit to Palm tf set");

    //LOADING HAND MODEL FOR JOINT NAMES (SHOULD WORK FOR ANY HAND)
    hand_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    hand_robot_model_ = hand_model_loader_->getModel();

    TemplateNodelet::loadGraspDatabaseXML(this->r_grasps_filename_, "right");

    ROS_INFO("Right Grasp Database loaded");

    // Load the left hand specific grasping database
    nh.getParam("/l_hand_tf/gp_T_hand", gp_T_hand);
    gp_T_hand_.setOrigin(tf::Vector3(static_cast<double>(gp_T_hand[0]),static_cast<double>(gp_T_hand[1]),static_cast<double>(gp_T_hand[2])));
    gp_T_hand_.setRotation(tf::Quaternion(static_cast<double>(gp_T_hand[3]),static_cast<double>(gp_T_hand[4]),static_cast<double>(gp_T_hand[5]),static_cast<double>(gp_T_hand[6])));

    ROS_INFO("Right Graspit to Palm tf set");

    TemplateNodelet::loadGraspDatabaseXML(this->l_grasps_filename_, "left");

    ROS_INFO("Left Grasp Database loaded");

    // Load the robot specific ghost_poses database
    TemplateNodelet::loadStandPosesDatabase(this->stand_filename_);

    ROS_INFO("Stand Poses Database loaded");
    
    id_counter_ = 0;

    timer = nh_out.createTimer(ros::Duration(0.033), &TemplateNodelet::timerCallback, this);
}

void TemplateNodelet::timerCallback(const ros::TimerEvent& event)
{
    this->publishTemplateList();
}


void TemplateNodelet::addTemplateCb(const flor_ocs_msgs::OCSTemplateAdd::ConstPtr& msg)
{
    std::cout << "Added template to list (id: " << (int)id_counter_ << ")" << std::endl;
    template_id_list_.push_back(id_counter_++);

    for (std::map<unsigned int,VigirObjectTemplate>::iterator it=object_template_map_.begin(); it!=object_template_map_.end(); ++it){
        if(it->second.path == (msg->template_path).substr(0, (msg->template_path).find_last_of(".")) ){ //removing file extension
            template_type_list_.push_back(it->second.type);	//Add the type of the template to be instantiated
            break;
        }
    }
    template_list_.push_back(msg->template_path);
    pose_list_.push_back(msg->pose);

    //ADD TEMPLATE TO PLANNING SCENE
    addCollisionObject(id_counter_-1,(msg->template_path).substr(0, (msg->template_path).find_last_of(".")),msg->pose.pose);

    this->publishTemplateList();
}

void TemplateNodelet::removeTemplateCb(const flor_ocs_msgs::OCSTemplateRemove::ConstPtr& msg)
{
    std::cout << "Removing template " << (unsigned int)msg->template_id << " from list... ";
    int index = 0;
    for(; index < template_id_list_.size(); index++)
        if(template_id_list_[index] == msg->template_id)
            break;
    if(index < template_id_list_.size())
    {
        std::cout << "Removed!" << std::endl;
        template_id_list_.erase(template_id_list_.begin()+index);
        template_type_list_.erase(template_type_list_.begin()+index);	//Remove it
        template_list_.erase(template_list_.begin()+index);
        pose_list_.erase(pose_list_.begin()+index);

        //REMOVE TEMPLATE FROM THE PLANING SCENE
        removeCollisionObject(msg->template_id);

        this->publishTemplateList();
    }
}

void TemplateNodelet::updateTemplateCb(const flor_ocs_msgs::OCSTemplateUpdate::ConstPtr& msg)
{
    std::cout << "Updating template " << (unsigned int)msg->template_id << "... ";
    int index = 0;
    for(; index < template_id_list_.size(); index++)
        if(template_id_list_[index] == msg->template_id)
            break;
    if(index < template_id_list_.size())
    {
        std::cout << "Updated!" << std::endl;
        pose_list_[index] = msg->pose;


        //UPDATE TEMPLATE POSE IN THE PLANNING SCENE
        moveCollisionObject(msg->template_id,msg->pose.pose);
    }
    this->publishTemplateList();
}

void TemplateNodelet::graspRequestCb(const flor_grasp_msgs::GraspSelection::ConstPtr& msg)
{
    std::cout << "Grasp request (id: " << msg->grasp_id << ")" << std::endl;
    {
    flor_grasp_msgs::GraspState cmd;

    //#typedef enum
    //#{
    //#    GRASP_MODE_NONE     = 0,
    //#    TEMPLATE_GRASP_MODE = 1,
    //#    MANUAL_GRASP_MODE   = 2,
    //#    NUM_GRASP_MODES
    //#
    //#} GraspControlModes;
    //#typedef enum
    //#{
    //#   GRASP_STATE_NONE   = 0, // unknown state
    //#    GRASP_INIT        = 1,
    //#    APPROACHING       = 2,
    //#    SURROUNDING       = 3,
    //#    GRASPING          = 4,
    //#    MONITORING        = 5,
    //#    OPENING           = 6,
    //#    GRASP_ERROR       = 7,
    //#    NUM_GRASP_STATES
    //#
    //#} GraspControlStates;

    unsigned char grasp_control_mode = 1;
    unsigned char grasp_control_state = 0;

    cmd.grasp_state.data = (grasp_control_mode <<4) + grasp_control_state;

    grasp_selected_state_pub_.publish(cmd);
    }
    {
    flor_grasp_msgs::GraspSelection cmd;

    cmd.template_id.data = msg->template_id.data;
    cmd.template_type.data = msg->template_type.data;
    cmd.grasp_id.data = msg->grasp_id.data;
    cmd.header.frame_id = "/world";
    cmd.header.stamp = ros::Time::now();

    grasp_selected_pub_.publish(cmd);
    }
}

void TemplateNodelet::graspStateFeedbackCb(const flor_grasp_msgs::GraspState::ConstPtr& msg)
{
    std::cout << "Grasp feedback" << std::endl;
    std::cout << "Grasp control mode" << ((msg->grasp_state.data & 0xf0) >> 4) << std::endl;
    std::cout << "Grasp control state" << (msg->grasp_state.data & 0x0f) << std::endl;
}

void TemplateNodelet::templateMatchFeedbackCb(const flor_grasp_msgs::TemplateSelection::ConstPtr& msg)
{
    std::cout << "Template feedback" << std::endl;
    int index = 0;
    for(; index < template_id_list_.size(); index++)
        if(template_id_list_[index] == msg->template_id.data)
            break;
    pose_list_[index] = msg->pose;
    publishTemplateList();
}

void TemplateNodelet::publishTemplateList()
{
    //std::cout << "timer" << std::endl;
    flor_ocs_msgs::OCSTemplateList cmd;

    cmd.template_id_list   = template_id_list_;
    cmd.template_list      = template_list_;
    cmd.template_type_list = template_type_list_;
    cmd.pose               = pose_list_;

    // publish complete list of templates and poses
    template_list_pub_.publish( cmd );

    // publish to TF
    ros::Time now = ros::Time::now();

    std::vector<tf::StampedTransform> transforms;

    for(int i = 0; i < pose_list_.size(); i++)
    {
        tf::StampedTransform template_pose_to_tf;

        template_pose_to_tf.frame_id_ = "/world";
        std::stringstream ss;
        ss << "/template_tf_" << (unsigned int)template_id_list_[i];
        template_pose_to_tf.child_frame_id_ = ss.str();

        template_pose_to_tf.stamp_ = now;

        const geometry_msgs::Point& vec_l (pose_list_[i].pose.position);
        template_pose_to_tf.setOrigin(tf::Vector3(vec_l.x, vec_l.y, vec_l.z));

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(pose_list_[i].pose.orientation, orientation);

        template_pose_to_tf.setRotation(orientation);

        transforms.push_back(template_pose_to_tf);
    }

    if(transforms.size() > 0)
        tfb_.sendTransform(transforms);
}

std::vector< std::vector <std::string> > TemplateNodelet::readCSVFile(std::string& file_name){
    std::ifstream file ( file_name.c_str() );
    if(!file){
        ROS_ERROR("NO DATABASE FILE FOUND: %s",file_name.c_str());
    }

    std::vector< std::vector <std::string> > db;
    for (std::string line; std::getline(file, line); )
    {
        std::istringstream in(line);
        std::vector<std::string> tmp;
        std::string value;

        while(std::getline(in, value, ',')) {
            std::stringstream trimmer;
            trimmer << value;
            value.clear();
            trimmer >> value; //removes white spaces
            tmp.push_back(value);
        }
        db.push_back(tmp);
    }
    return db;
}

void TemplateNodelet::loadGraspDatabaseXML(std::string& file_name, std::string hand_side)
{
    //Getting joints for hand from URDF robot description
    if(hand_robot_model_->hasJointModelGroup(hand_side+"_hand"))
    {
        hand_joint_names_.clear();
        hand_joint_names_ = hand_robot_model_->getJointModelGroup(hand_side+"_hand")->getActiveJointModelNames();
    }else{
        ROS_WARN("NO JOINTS FOUND FOR %s HAND",hand_side.c_str());
    }

    ROS_INFO("%s %s hand model gotten, #actuated joints: %ld ",hand_side.c_str(), hand_robot_model_->getName().c_str(),hand_joint_names_.size() );

    for(int i = 0; i < hand_joint_names_.size(); i++)
        ROS_INFO("Joint %d: %s",i,hand_joint_names_[i].c_str());

    //Creating XML document from parameter server string of grasp library
    TiXmlDocument doc;
    doc.Parse((const char*)file_name.c_str(), 0, TIXML_ENCODING_UTF8);
    if (doc.ErrorId() != 0){
        ROS_ERROR("Could not read file for %s hand",hand_side.c_str());
        return;
    }

    TiXmlHandle hDoc(&doc);
    TiXmlElement* pElem;
    TiXmlHandle hRoot(0);

    pElem=hDoc.FirstChildElement().Element();
    // should always have a valid root but handle gracefully if it does
    if (!pElem){
        ROS_ERROR("File for %s hand read but empty", hand_side.c_str());
        return;
    }

    // save this for later
    hRoot=TiXmlHandle(pElem);
    ROS_INFO("Reading %s for %s hand", pElem->Value(), hand_side.c_str());

    TiXmlElement* pGrasps=hRoot.FirstChild( "grasps" ).Element();
    for( pGrasps; pGrasps; pGrasps=pGrasps->NextSiblingElement()) //Iterates thorugh all template types
    {
        const char *pName=pGrasps->Attribute("template_name");
        int template_type;
        pGrasps->QueryIntAttribute("template_type", &template_type);
        if (pName) ROS_INFO("Reading Grasps for %s type: %d",pName, template_type);

        TiXmlElement* pGrasp=pGrasps->FirstChildElement( "grasp" );
        for( pGrasp; pGrasp; pGrasp=pGrasp->NextSiblingElement( "grasp" ) )   //Iterates thorugh all grasp IDs for this particular template type
        {
            moveit_msgs::Grasp grasp;
            float x,y,z,qx,qy,qz,qw;
            grasp.grasp_pose.header.frame_id = hand_side[0]+"_hand";

            const char *pID=pGrasp->Attribute("id");
            if (pID) ROS_INFO("Found Grasp id: %s",pID);
            grasp.id = std::string(pID);

            TiXmlElement* pPose=pGrasp->FirstChildElement( "final_pose" );       //Gets final grasp pose
            if(!pPose){
                ROS_WARN("Grasp ID: %s does not contain an final pose, setting identity",pID);
                grasp.grasp_pose.pose.position.x    = 0.0;
                grasp.grasp_pose.pose.position.y    = 0.0;
                grasp.grasp_pose.pose.position.z    = 0.0;
                grasp.grasp_pose.pose.orientation.x = 0.0;
                grasp.grasp_pose.pose.orientation.y = 0.0;
                grasp.grasp_pose.pose.orientation.z = 0.0;
                grasp.grasp_pose.pose.orientation.w = 1.0;
            }else{
                pPose->QueryFloatAttribute("x",  &x);
                pPose->QueryFloatAttribute("y",  &y);
                pPose->QueryFloatAttribute("z",  &z);
                pPose->QueryFloatAttribute("qx", &qx);
                pPose->QueryFloatAttribute("qy", &qy);
                pPose->QueryFloatAttribute("qz", &qz);
                pPose->QueryFloatAttribute("qw", &qw);

                grasp.grasp_pose.pose.position.x    = x;
                grasp.grasp_pose.pose.position.y    = y;
                grasp.grasp_pose.pose.position.z    = z;
                grasp.grasp_pose.pose.orientation.x = qx;
                grasp.grasp_pose.pose.orientation.y = qy;
                grasp.grasp_pose.pose.orientation.z = qz;
                grasp.grasp_pose.pose.orientation.w = qw;

                grasp.grasp_pose.header.frame_id    = hand_side[0]+"_hand";
            }

            ROS_INFO_STREAM("Added grasp information id: " << grasp.id << " pose: " << std::endl << grasp.grasp_pose.pose);

            TiXmlElement* pApproachingVector=pGrasp->FirstChildElement( "approaching_vector" );       //Gets approaching vector
            if(!pApproachingVector){
                ROS_WARN("Grasp ID: %s does not contain an approaching vector, setting default values",pID);
                grasp.pre_grasp_approach.direction.vector.x = 0.00;
                grasp.pre_grasp_approach.direction.vector.y = 1.00;
                grasp.pre_grasp_approach.direction.vector.z = 0.00;
                grasp.pre_grasp_approach.desired_distance   = 0.20;
                grasp.pre_grasp_approach.min_distance       = 0.05;
            }else{
                grasp.pre_grasp_approach.direction.header.frame_id = hand_side[0]+"_hand"; //Should r_hand or l_hand also be a parameter?

                pApproachingVector->QueryFloatAttribute("x", &x);
                pApproachingVector->QueryFloatAttribute("y", &y);
                pApproachingVector->QueryFloatAttribute("z", &z);

                grasp.pre_grasp_approach.direction.vector.x = x;
                grasp.pre_grasp_approach.direction.vector.y = y;
                grasp.pre_grasp_approach.direction.vector.z = z;

                pApproachingVector->QueryFloatAttribute("desired", &grasp.pre_grasp_approach.desired_distance);
                pApproachingVector->QueryFloatAttribute("minimal", &grasp.pre_grasp_approach.min_distance);
            }

            ROS_INFO_STREAM("Added aproaching vector information id: " << grasp.id << " pose: " << std::endl << grasp.pre_grasp_approach);

            grasp.pre_grasp_posture.points.resize(1);
            grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(3.0);

            TiXmlElement* pPrePosture=pGrasp->FirstChildElement( "pre_grasp_posture" );
            if(!pPrePosture){
                ROS_WARN("Grasp ID: %s does not contain a pregrasp posture, setting all %d joints to zeros",pID, (int)hand_joint_names_.size());
                if(hand_joint_names_.size() > 0){
                    grasp.pre_grasp_posture.joint_names.resize(hand_joint_names_.size());
                    for(int j=0; j<hand_joint_names_.size();j++)
                        grasp.pre_grasp_posture.joint_names[j] = hand_joint_names_.at(j);
                    grasp.pre_grasp_posture.points[0].positions.resize(hand_joint_names_.size());
                    for(int j=0; j<hand_joint_names_.size();j++)
                        grasp.pre_grasp_posture.points[0].positions[j] = 0.0;  //Setting default joint values to zeros
                }else{
                    ROS_WARN("Grasp ID: %s does not contain a pregrasp posture and URDF shows no %s hand joints",pID, hand_side.c_str());
                }
            }else{
                TiXmlElement* pFinger=pPrePosture->FirstChildElement( "finger" );       //Gets pre finger joints
                if(!pFinger){
                    ROS_WARN("Grasp ID: %s does not contain any finger, setting joints to zeros",pID);
                }else{
                    for( pFinger; pFinger; pFinger=pFinger->NextSiblingElement())   //Iterates thorugh all fingers for this particular pre posture
                    {
                        TiXmlElement* pJoint=pFinger->FirstChildElement( "joint" );       //Gets approaching vector
                        if(!pJoint){
                            ROS_WARN("Grasp ID: %s does not contain joints for finger %s",pID, pFinger->Attribute("idx"));
                        }else{
                            for( pJoint; pJoint; pJoint=pJoint->NextSiblingElement())   //Iterates thorugh all fingers for this particular pre posture
                            {
                                const char *pJointName=pJoint->Attribute("name");
                                if (pJointName)
                                    grasp.pre_grasp_posture.joint_names.push_back(pJointName);
                                else
                                    ROS_WARN("Found joint without name for finger %s",pFinger->Attribute("idx"));
                                pJoint->QueryFloatAttribute("value", &x);
                                grasp.pre_grasp_posture.points[0].positions.push_back(x);
                            }
                        }
                    }

                    ROS_INFO_STREAM("Added pre_grasp_posture information id: " << grasp.id << " pose: " << std::endl << grasp.pre_grasp_posture);
                }
            }

            grasp.grasp_posture.points.resize(1);
            grasp.grasp_posture.points[0].time_from_start = ros::Duration(3.0);

            TiXmlElement* pPosture=pGrasp->FirstChildElement( "grasp_posture" );
            if(!pPosture){
                ROS_WARN("Grasp ID: %s does not contain a grasp posture, setting all %d joints to zeros",pID, (int)hand_joint_names_.size());
                if(hand_joint_names_.size() > 0){
                    grasp.grasp_posture.joint_names.resize(hand_joint_names_.size());
                    for(int j=0; j<hand_joint_names_.size();j++)
                        grasp.grasp_posture.joint_names[j] = hand_joint_names_.at(j);
                    grasp.grasp_posture.points[0].positions.resize(hand_joint_names_.size());
                    for(int j=0; j<hand_joint_names_.size();j++)
                        grasp.grasp_posture.points[0].positions[j] = 0.0;  //Setting default joint values to zeros
                }else{
                    ROS_WARN("Grasp ID: %s does not contain a grasp posture and URDF shows no %s hand joints",pID, hand_side.c_str());
                }
            }else{
                TiXmlElement* pFinger=pPosture->FirstChildElement( "finger" );       //Gets final finger joints
                if(!pFinger){
                    ROS_WARN("Grasp ID: %s does not contain any finger, setting joints to zeros",pID);
                }else{
                    for( pFinger; pFinger; pFinger=pFinger->NextSiblingElement())   //Iterates thorugh all fingers for this particular posture
                    {
                        TiXmlElement* pJoint=pFinger->FirstChildElement( "joint" );       //Gets approaching vector
                        if(!pJoint){
                            ROS_WARN("Grasp ID: %s does not contain joints for finger %s",pID, pFinger->Attribute("idx"));
                        }else{
                            for( pJoint; pJoint; pJoint=pJoint->NextSiblingElement())   //Iterates thorugh all fingers for this particular posture
                            {
                                const char *pJointName=pJoint->Attribute("name");
                                if (pJointName)
                                    grasp.grasp_posture.joint_names.push_back(pJointName);
                                else
                                    ROS_WARN("Found joint without name for finger %s",pFinger->Attribute("idx"));
                                pJoint->QueryFloatAttribute("value", &x);
                                grasp.grasp_posture.points[0].positions.push_back(x);
                            }
                        }
                    }

                    ROS_INFO_STREAM("Added grasp_posture information id: " << grasp.id << " pose: " << std::endl << grasp.grasp_posture);
                }
            }

            //RETREAT VECTOR (FIXING TO LIFT 10cm AFTER GRASPING)
            //ROS_INFO("Staring retreat vector idx: %d",idx);
            grasp.post_grasp_retreat.direction.header.frame_id = "world";
            grasp.post_grasp_retreat.direction.vector.z        = 1.0;
            grasp.post_grasp_retreat.min_distance              = 0.05;
            grasp.post_grasp_retreat.desired_distance          = 0.1;

            if(object_template_map_.find(template_type) != object_template_map_.end())   //Template Type exists
                object_template_map_[template_type].grasps.insert(std::pair<unsigned int,moveit_msgs::Grasp>(std::atoi(grasp.id.c_str()),grasp));
        }
    }
    for (std::map<unsigned int,VigirObjectTemplate>::iterator it=object_template_map_.begin(); it!=object_template_map_.end(); ++it)
        for (std::map<unsigned int,moveit_msgs::Grasp>::iterator it2=it->second.grasps.begin(); it2!=it->second.grasps.end(); ++it2)
            ROS_INFO("OT Map, inside ot: %d -> Grasp id %s ", it->second.type, it2->second.id.c_str());
}

void TemplateNodelet::loadStandPosesDatabase(std::string& file_name){
    /*
     * Need to fill object_template_map_[type].stand_poses with the poses read from file
     *
     * ORDER IN FILE SHOULD MATCH ORDER IN READING
     * template type,
     * stand pose id,
     * stand pose relative to template (x,y,z,qw,qx,qy,qz),
    */
    ROS_INFO("Loading Stand Poses...");
   std::vector <std::vector <std::string> > db = readCSVFile(file_name);
   
	unsigned int template_type;
	unsigned int stand_pose_id;
	unsigned int j;
    vigir_object_template_msgs::StandPose current_pose;
	std::map<unsigned int, VigirObjectTemplate>::iterator current_template;
	for (unsigned int i = 1; i < db.size(); ++i) {
        template_type                        = std::atoi(db[i][0].c_str());
        stand_pose_id                        = std::atoi(db[i][1].c_str());
        current_pose.id                      = stand_pose_id;
        current_pose.pose.header.frame_id    = "/world";
        current_pose.pose.header.stamp       = ros::Time::now();

        current_pose.pose.pose.position.x    = std::atof(db[i][2].c_str());
        current_pose.pose.pose.position.y    = std::atof(db[i][3].c_str());
        current_pose.pose.pose.position.z    = std::atof(db[i][4].c_str());

        current_pose.pose.pose.orientation.w = std::atof(db[i][5].c_str());
        current_pose.pose.pose.orientation.x = std::atof(db[i][6].c_str());
        current_pose.pose.pose.orientation.y = std::atof(db[i][7].c_str());
        current_pose.pose.pose.orientation.z = std::atof(db[i][8].c_str());

		current_template = object_template_map_.find(template_type);
		if (current_template == object_template_map_.end()){
			ROS_WARN_STREAM("Could not find associated template for stand pose. Template: "
									<< template_type << " stand pose: " << stand_pose_id 
									<< ". Adding stand pose regardless");
			VigirObjectTemplate new_template;
			current_template = object_template_map_.insert(std::pair<unsigned int, VigirObjectTemplate> (template_type, new_template)).first;
		}
		
		if (current_template->second.stand_poses.find(stand_pose_id) != current_template->second.stand_poses.end()) {
			ROS_WARN_STREAM("Duplicates in the stand pose list! Template " << template_type 
									<< " has two stand poses of id: " << stand_pose_id << ". Ignoring second.");
			continue;
		}
		
        current_template->second.stand_poses.insert(std::pair<unsigned int, vigir_object_template_msgs::StandPose> (stand_pose_id, current_pose));
        ROS_INFO_STREAM("Added stand information: type " << template_type << " pose id: " << stand_pose_id << " pose " << current_pose.pose);
   }
}


void TemplateNodelet::loadObjectTemplateDatabase(std::string& file_name)
{

    std::vector< std::vector <std::string> > db = readCSVFile(file_name);

    for(int i = 1; i < db.size(); i++) //STARTING FROM 1 SINCE FIRST LINE IS HEADER BEGINING WITH "#"
    {
        VigirObjectTemplate object_template;
        unsigned int type = std::atoi(db[i][0].c_str());
        std::string  path = db[i][1];

        geometry_msgs::Point b_max;
        geometry_msgs::Point b_min;
        b_min.x = std::atof(db[i][2].c_str());
        b_min.y = std::atof(db[i][3].c_str());
        b_min.z = std::atof(db[i][4].c_str());
        b_max.x = std::atof(db[i][5].c_str());
        b_max.y = std::atof(db[i][6].c_str());
        b_max.z = std::atof(db[i][7].c_str());

        geometry_msgs::Point com ;
        com.x = std::atof(db[i][8].c_str());
        com.y = std::atof(db[i][9].c_str());
        com.z = std::atof(db[i][10].c_str());

        double mass = std::atof(db[i][11].c_str());

        object_template.b_max = b_max;
        object_template.b_min = b_min;
        object_template.com   = com;
        object_template.mass  = mass;
        object_template.id    = i-1;
        object_template.type  = type;
        object_template.path  = path;
        object_template.name  = path.substr(path.find_last_of("/")+1,path.size());
        object_template_map_.insert(std::pair<unsigned int,VigirObjectTemplate>(type,object_template));
        ROS_INFO(" Inserting Object template type: %d with id: %d, name %s", object_template_map_[type].type, object_template_map_[type].id, object_template_map_[type].name.c_str());
    }
}

void TemplateNodelet::gripperTranslationToPreGraspPose(geometry_msgs::Pose& pose, moveit_msgs::GripperTranslation& trans){
    geometry_msgs::Vector3Stamped direction = trans.direction;
    tf::Transform template_T_hand, vec_in, vec_out;
    ROS_INFO("receiving trans distance: %f; dx: %f, dy: %f, dz: %f", trans.desired_distance, direction.vector.x, direction.vector.y, direction.vector.z);
    float norm = sqrt((direction.vector.x * direction.vector.x) +(direction.vector.y * direction.vector.y) +(direction.vector.z * direction.vector.z));
    if(norm != 0){
        direction.vector.x /= norm;
        direction.vector.y /= norm;
        direction.vector.z /= norm;
    }else{
        ROS_INFO("Norm is ZERO!");
        direction.vector.x = 0 ;
        direction.vector.y = -1;
        direction.vector.z = 0 ;
    }

    direction.vector.x *= -trans.desired_distance;
    direction.vector.y *= -trans.desired_distance;
    direction.vector.z *= -trans.desired_distance;

    ROS_INFO("setting trans; dx: %f, dy: %f, dz: %f", direction.vector.x, direction.vector.y, direction.vector.z);

    template_T_hand.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
    template_T_hand.setOrigin(tf::Vector3(0,0,0));
    vec_in.setOrigin(tf::Vector3(direction.vector.x,direction.vector.y,direction.vector.z));
    vec_in.setRotation(tf::Quaternion(0,0,0,1));

    vec_out = template_T_hand * vec_in;

    ROS_INFO("setting result; dx: %f, dy: %f, dz: %f", vec_out.getOrigin().getX(), vec_out.getOrigin().getY(), vec_out.getOrigin().getZ());

    pose.position.x += vec_out.getOrigin().getX();
    pose.position.y += vec_out.getOrigin().getY();
    pose.position.z += vec_out.getOrigin().getZ();
}

bool TemplateNodelet::templateInfoSrv(vigir_object_template_msgs::GetTemplateStateAndTypeInfo::Request& req,
                                      vigir_object_template_msgs::GetTemplateStateAndTypeInfo::Response& res)
{
    /*Fill in the blanks of the response "res"
     * with the info of the template id in the request "req"
    */
       //Find the template
	unsigned int index = 0;
    unsigned int template_id;
	for(; index < template_id_list_.size(); index++) {
        if(template_type_list_[index] == req.template_type){
            template_id = template_type_list_[index];
            ROS_INFO("Template Type found in server, index: %d, list: %d, requested: %d",index, template_type_list_[index], req.template_type);
            break;
		}
    }
	
    if (index >= template_id_list_.size()){
        //ROS_ERROR_STREAM("Service requested template id " << req.template_type.data << " when no such id has been instantiated. Callback returning false.");
        ROS_ERROR("Service requested template type %d when no such type has been instantiated. Callback returning false.",req.template_type);
		return false;
    }

    res.template_state_information.template_type = req.template_type;
    res.template_state_information.template_id   = template_id;
    res.template_state_information.type_name 	 = object_template_map_[req.template_type].name;
    res.template_state_information.pose 		 = pose_list_[index];

    res.template_type_information.type_name      = object_template_map_[req.template_type].name;
    res.template_type_information.mass           = object_template_map_[req.template_type].mass;
    res.template_type_information.center_of_mass = object_template_map_[req.template_type].com;
    res.template_type_information.b_max          = object_template_map_[req.template_type].b_max;
    res.template_type_information.b_min          = object_template_map_[req.template_type].b_min;
	
	//Transfer all known grasps to response
    for (std::map<unsigned int,moveit_msgs::Grasp>::iterator it =  object_template_map_[req.template_type].grasps.begin();
                                                             it != object_template_map_[req.template_type].grasps.end();
                                                             ++it) {
        res.template_type_information.grasps.push_back(it->second);
    }

    //Transfer all known stand poses to response
    for (std::map<unsigned int,vigir_object_template_msgs::StandPose>::iterator it = object_template_map_[req.template_type].stand_poses.begin(); it != object_template_map_[req.template_type].stand_poses.end(); ++it) {
        res.template_type_information.stand_poses.push_back(it->second);
    }

	//Compose a mesh marker
	res.template_type_information.geometry_marker.header.frame_id = "/world";
    res.template_type_information.geometry_marker.header.stamp    = ros::Time::now();
    res.template_type_information.geometry_marker.type            = res.template_type_information.geometry_marker.MESH_RESOURCE;
    res.template_type_information.geometry_marker.action          = res.template_type_information.geometry_marker.ADD;
    res.template_type_information.geometry_marker.scale.x         = 1;
    res.template_type_information.geometry_marker.scale.y         = 1;
    res.template_type_information.geometry_marker.scale.z         = 1;
    res.template_type_information.geometry_marker.lifetime        = ros::Duration(0);
    res.template_type_information.geometry_marker.frame_locked    = true;
    res.template_type_information.geometry_marker.mesh_resource   = object_template_map_[req.template_type].path;
    res.template_type_information.geometry_marker.pose            = pose_list_[index].pose;

  return true;
}

bool TemplateNodelet::graspInfoSrv(vigir_object_template_msgs::GetGraspInfo::Request& req,
                                   vigir_object_template_msgs::GetGraspInfo::Response& res)
{
    /*Fill in the blanks of the response "res"
     * with the info of the template id in the request "req"
    */
    int i=0;
    for (std::map<unsigned int,moveit_msgs::Grasp>::iterator it2  = object_template_map_[req.template_type].grasps.begin();
                                                             it2 != object_template_map_[req.template_type].grasps.end();
                                                             ++it2, i++){
        res.grasp_information.grasps.push_back(it2->second);
    }
    i=0;
    for (std::map<unsigned int,vigir_object_template_msgs::StandPose>::iterator it2  = object_template_map_[req.template_type].stand_poses.begin();
                                                                     it2 != object_template_map_[req.template_type].stand_poses.end();
                                                                     ++it2, i++){
        res.grasp_information.stand_poses.push_back(it2->second);
    }
    return true;
}

bool TemplateNodelet::attachObjectTemplateSrv(vigir_object_template_msgs::SetAttachedObjectTemplate::Request& req,
                                              vigir_object_template_msgs::SetAttachedObjectTemplate::Response& res)
{

    /* First, define the REMOVE object message*/
    moveit_msgs::CollisionObject remove_object;
    remove_object.id              = boost::to_string(req.template_id);
    remove_object.header.frame_id = "world";
    remove_object.operation       = remove_object.REMOVE;
    co_pub_.publish(remove_object);
    ROS_INFO("Collision object :%s removed",remove_object.id.c_str());


    ROS_INFO("Attach template to %s started... ",req.pose.header.frame_id.c_str());

    // Define the attached object message
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // We will use this message to add or
    // subtract the object from the world
    // and to attach the object to the robot
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name              = req.pose.header.frame_id;
    /* The header must contain a valid TF frame*/
    attached_object.object.header.frame_id = req.pose.header.frame_id;
    /* The id of the object */
    attached_object.object.id              = boost::to_string(req.template_id);

    unsigned int index = 0;
    std::string mesh_name;
    geometry_msgs::PoseStamped template_pose;

    for(; index < template_id_list_.size(); index++) {
        if(template_id_list_[index] == req.template_id){
            mesh_name = (template_list_[index]).substr(0, (template_list_[index]).find_last_of("."));
            template_pose      = pose_list_[index];
            ROS_INFO("Template %s found in server, index: %d, list: %d, requested: %d",mesh_name.c_str(), index, template_id_list_[index], req.template_id);
            break;
        }
    }

    if (index >= template_id_list_.size()){
        //ROS_ERROR_STREAM("Service requested template id " << req.template_type.data << " when no such id has been instantiated. Callback returning false.");
        ROS_ERROR("Service requested template id %d when no such id has been instantiated. Callback returning false.",req.template_id);
        return false;
    }

    std::string mesh_path = "package://vigir_template_library/object_templates/"+ mesh_name + ".ply";
    ROS_INFO("Requesting mesh_path: %s", mesh_path.c_str());


    shapes::Mesh* shape = shapes::createMeshFromResource(mesh_path);
    shapes::ShapeMsg mesh_msg;

    shapes::constructMsgFromShape(shape,mesh_msg);

    shape_msgs::Mesh mesh_;
    mesh_ = boost::get<shape_msgs::Mesh>(mesh_msg);

    tf::Transform wt_pose;
    tf::Transform tp_pose;
    tf::Transform target_pose;
    wt_pose.setRotation(tf::Quaternion(req.pose.pose.orientation.x,req.pose.pose.orientation.y,req.pose.pose.orientation.z,req.pose.pose.orientation.w));
    wt_pose.setOrigin(tf::Vector3(req.pose.pose.position.x,req.pose.pose.position.y,req.pose.pose.position.z) );
    tp_pose.setRotation(tf::Quaternion(template_pose.pose.orientation.x,template_pose.pose.orientation.y,template_pose.pose.orientation.z,template_pose.pose.orientation.w));
    tp_pose.setOrigin(tf::Vector3(template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z) );

    target_pose = wt_pose.inverse() * tp_pose;

    geometry_msgs::Pose pose;
    pose.orientation.x = target_pose.getRotation().getX();
    pose.orientation.y = target_pose.getRotation().getY();
    pose.orientation.z = target_pose.getRotation().getZ();
    pose.orientation.w = target_pose.getRotation().getW();
    pose.position.x    = target_pose.getOrigin().getX();
    pose.position.y    = target_pose.getOrigin().getY();
    pose.position.z    = target_pose.getOrigin().getZ();

    attached_object.object.meshes.push_back(mesh_);
    attached_object.object.mesh_poses.push_back(pose);

    // Note that attaching an object to the robot requires
    // the corresponding operation to be specified as an ADD operation
    attached_object.object.operation = attached_object.object.ADD;

    std::string hand_side;
    if(req.pose.header.frame_id == "r_hand")
        hand_side = "right";
    else
        hand_side = "left";

    hand_link_names_ = hand_robot_model_->getJointModelGroup(hand_side+ "_hand")->getLinkModelNames();
    attached_object.touch_links.push_back(hand_robot_model_->getJointModelGroup(hand_side+ "_hand")->getParentModel().getLinkModelNames().at(0));
    for(int i = 0; i < hand_link_names_.size(); i++){
        ROS_INFO("Link %d: %s",i,hand_link_names_[i].c_str());
        attached_object.touch_links.push_back(hand_link_names_[i]);
    }

    ROS_INFO("Attaching the object to the %s link", req.pose.header.frame_id.c_str());
    aco_pub_.publish(attached_object);

    return true;
}

bool TemplateNodelet::stitchObjectTemplateSrv(vigir_object_template_msgs::SetAttachedObjectTemplate::Request& req,
                                              vigir_object_template_msgs::SetAttachedObjectTemplate::Response& res)
{
    /* First, define the DETACH object message*/
    moveit_msgs::AttachedCollisionObject tmp_attached_object;
    tmp_attached_object.object.id = boost::to_string(req.template_id);
    tmp_attached_object.object.operation = tmp_attached_object.object.REMOVE;

    ROS_INFO("Dettaching the object %s",tmp_attached_object.object.id.c_str());
    aco_pub_.publish(tmp_attached_object);


    tmp_attached_object.link_name              = req.pose.header.frame_id;
    /* The header must contain a valid TF frame*/
    tmp_attached_object.object.header.frame_id = req.pose.header.frame_id;

    unsigned int index = 0;
    std::string mesh_name;
    geometry_msgs::PoseStamped template_pose;

    for(; index < template_id_list_.size(); index++) {
        if(template_id_list_[index] == req.template_id){
            mesh_name = (template_list_[index]).substr(0, (template_list_[index]).find_last_of("."));
            template_pose      = pose_list_[index];
            ROS_INFO("Template %s found in server, index: %d, list: %d, requested: %d",mesh_name.c_str(), index, template_id_list_[index], req.template_id);
            break;
        }
    }

    if (index >= template_id_list_.size()){
        //ROS_ERROR_STREAM("Service requested template id " << req.template_type.data << " when no such id has been instantiated. Callback returning false.");
        ROS_ERROR("Service requested template id %d when no such id has been instantiated. Callback returning false.",req.template_id);
        return false;
    }

    std::string mesh_path = "package://vigir_template_library/object_templates/"+ mesh_name + ".ply";
    ROS_INFO("Requesting mesh_path: %s", mesh_path.c_str());


    shapes::Mesh* shape = shapes::createMeshFromResource(mesh_path);
    shapes::ShapeMsg mesh_msg;

    shapes::constructMsgFromShape(shape,mesh_msg);

    shape_msgs::Mesh mesh_;
    mesh_ = boost::get<shape_msgs::Mesh>(mesh_msg);

    tf::Transform wt_pose;
    tf::Transform tp_pose;
    tf::Transform target_pose;
    wt_pose.setRotation(tf::Quaternion(req.pose.pose.orientation.x,req.pose.pose.orientation.y,req.pose.pose.orientation.z,req.pose.pose.orientation.w));
    wt_pose.setOrigin(tf::Vector3(req.pose.pose.position.x,req.pose.pose.position.y,req.pose.pose.position.z) );
    tp_pose.setRotation(tf::Quaternion(template_pose.pose.orientation.x,template_pose.pose.orientation.y,template_pose.pose.orientation.z,template_pose.pose.orientation.w));
    tp_pose.setOrigin(tf::Vector3(template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z) );

    target_pose = wt_pose.inverse() * tp_pose;

    geometry_msgs::Pose pose;
    pose.orientation.x = target_pose.getRotation().getX();
    pose.orientation.y = target_pose.getRotation().getY();
    pose.orientation.z = target_pose.getRotation().getZ();
    pose.orientation.w = target_pose.getRotation().getW();
    pose.position.x    = target_pose.getOrigin().getX();
    pose.position.y    = target_pose.getOrigin().getY();
    pose.position.z    = target_pose.getOrigin().getZ();

    tmp_attached_object.object.meshes.push_back(mesh_);
    tmp_attached_object.object.mesh_poses.push_back(pose);

    // Note that attaching an object to the robot requires
    // the corresponding operation to be specified as an ADD operation
    tmp_attached_object.object.operation = tmp_attached_object.object.ADD;

    ros::Duration(0.5).sleep();

    ROS_INFO("Removing collision object :%s started",tmp_attached_object.object.id.c_str());
    /* First, define the REMOVE object message*/
    moveit_msgs::CollisionObject remove_object;
    remove_object.id = tmp_attached_object.object.id;
    remove_object.header.frame_id = "world";
    remove_object.operation = remove_object.REMOVE;

    ROS_INFO("Collision object :%s removed",remove_object.id.c_str());

    co_pub_.publish(remove_object);

    ROS_INFO("Stitching the object %s ",tmp_attached_object.object.id.c_str());
    aco_pub_.publish(tmp_attached_object);
    return true;

}

bool TemplateNodelet::detachObjectTemplateSrv(vigir_object_template_msgs::DetachObjectTemplate::Request& req,
                                              vigir_object_template_msgs::DetachObjectTemplate::Response& res)
{
    /* First, define the DETACH object message*/
    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = boost::to_string(req.template_id);
    detach_object.object.operation = detach_object.object.REMOVE;

    ROS_INFO("Dettaching the object %s",detach_object.object.id.c_str());
    aco_pub_.publish(detach_object);
}

// transform endeffort to palm pose used by GraspIt
int TemplateNodelet::staticTransform(geometry_msgs::Pose& palm_pose)
{
    tf::Transform o_T_hand;    //describes hand in object's frame
    tf::Transform o_T_pg;       //describes palm_from_graspit in object's frame

    o_T_pg.setRotation(tf::Quaternion(palm_pose.orientation.x,palm_pose.orientation.y,palm_pose.orientation.z,palm_pose.orientation.w));
    o_T_pg.setOrigin(tf::Vector3(palm_pose.position.x,palm_pose.position.y,palm_pose.position.z) );

    o_T_hand = o_T_pg * gp_T_hand_;

    tf::Quaternion hand_quat;
    tf::Vector3    hand_vector;
    hand_quat   = o_T_hand.getRotation();
    hand_vector = o_T_hand.getOrigin();

    palm_pose.position.x    = hand_vector.getX();
    palm_pose.position.y    = hand_vector.getY();
    palm_pose.position.z    = hand_vector.getZ();
    palm_pose.orientation.x = hand_quat.getX();
    palm_pose.orientation.y = hand_quat.getY();
    palm_pose.orientation.z = hand_quat.getZ();
    palm_pose.orientation.w = hand_quat.getW();
    return 0;
}


void TemplateNodelet::addCollisionObject(int index, std::string mesh_name, geometry_msgs::Pose pose){
    //Add collision object with template pose and bounding box

    ROS_INFO("Add collision template started... ");
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = boost::to_string((unsigned int)index);
    collision_object.header.frame_id = "world";

    std::string path = ros::package::getPath("vigir_template_library") + "/object_templates/" + mesh_name + ".ply";

    std::ifstream infile(path.c_str());
    if(infile.good()){
        std::string mesh_path = "package://vigir_template_library/object_templates/"+mesh_name + ".ply";
        ROS_INFO("1 mesh_path: %s", mesh_path.c_str());
        shapes::Mesh* shape = shapes::createMeshFromResource(mesh_path);
        shapes::ShapeMsg mesh_msg;

        shapes::constructMsgFromShape(shape,mesh_msg);
        shape_msgs::Mesh                        mesh_;
        mesh_ = boost::get<shape_msgs::Mesh>(mesh_msg);

//        collision_object.primitives.resize(1);
//        collision_object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
//        collision_object.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
//        collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.10;
//        collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.00;
//        collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
//        collision_object.primitive_poses.push_back(pose);


        collision_object.meshes.push_back(mesh_);
        collision_object.mesh_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;
        ROS_INFO("Adding the object: %s to the environment",collision_object.id.c_str());
        co_pub_.publish(collision_object);
    }else{
        ROS_ERROR("Mesh file: %s doesn't exists",path.c_str());
    }
}

void TemplateNodelet::moveCollisionObject(int index, geometry_msgs::Pose pose){
    //Add collision object with template pose and bounding box

    ROS_INFO("Move collision template started... ");
    moveit_msgs::CollisionObject collision_object;
    collision_object.id              = boost::to_string((unsigned int)index);
    collision_object.header.frame_id = "world";
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation       = collision_object.MOVE;
    ROS_INFO("Moving the object in the environment");
    co_pub_.publish(collision_object);
}

void TemplateNodelet::removeCollisionObject(int index){
    //Add collision object with template pose and bounding box

    ROS_INFO("Remove collision template started... ");
    moveit_msgs::CollisionObject collision_object;
    collision_object.id              = boost::to_string((unsigned int)index);
    collision_object.header.frame_id = "world";
    collision_object.operation       = collision_object.REMOVE;
    ROS_INFO("Removing the object %d from the environment", index);
    co_pub_.publish(collision_object);
}



}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_template_nodelet, TemplateNodelet, ocs_template::TemplateNodelet, nodelet::Nodelet);
