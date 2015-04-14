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
    template_snap_sub_           = nh_out.subscribe<flor_grasp_msgs::TemplateSelection>( "snap", 1, &TemplateNodelet::snapTemplateCb, this );
    template_match_feedback_sub_ = nh_out.subscribe<flor_grasp_msgs::TemplateSelection>( "template_match_feedback", 1, &TemplateNodelet::templateMatchFeedbackCb, this );
    grasp_request_sub_           = nh_out.subscribe<flor_grasp_msgs::GraspSelection>( "grasp_request", 1, &TemplateNodelet::graspRequestCb, this );
    grasp_state_feedback_sub_    = nh_out.subscribe<flor_grasp_msgs::GraspState>( "grasp_state_feedback", 1, &TemplateNodelet::graspStateFeedbackCb, this );

    template_info_server_        = nh_out.advertiseService("/template_info", &TemplateNodelet::templateInfoSrv, this);

    grasp_info_server_           = nh_out.advertiseService("/grasp_info", &TemplateNodelet::graspInfoSrv, this);

    inst_grasp_info_server_      = nh_out.advertiseService("/instantiated_grasp_info", &TemplateNodelet::instantiatedGraspInfoSrv, this);

    attach_object_server_        = nh_out.advertiseService("/attach_object_template", &TemplateNodelet::attachObjectTemplateSrv, this);
    stitch_object_server_        = nh_out.advertiseService("/stitch_object_template", &TemplateNodelet::stitchObjectTemplateSrv, this);
    detach_object_server_        = nh_out.advertiseService("/detach_object_template", &TemplateNodelet::detachObjectTemplateSrv, this);

    co_pub_                      = nh_out.advertise<moveit_msgs::CollisionObject>("/collision_object", 1, false);
    aco_pub_                     = nh_out.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 1, false);

    ROS_INFO(" Start reading database files");

    //Get hand parameters from server
    left_wrist_link_ = "l_hand";
    left_palm_link_  = "l_palm";
    left_hand_group_ = "l_hand_group";
    right_wrist_link_ = "r_hand";
    right_palm_link_  = "r_palm";
    right_hand_group_ = "r_hand_group";

    if(!nhp.getParam("/left_wrist_link", left_wrist_link_))
        ROS_WARN("No left wrist link defined, using l_hand as default");
    if(!nhp.getParam("/left_palm_link",  left_palm_link_))
        ROS_WARN("No left palm link defined, using l_palm as default");
    if(!nhp.getParam("/left_hand_group", left_hand_group_))
        ROS_WARN("No left hand group defined, using l_hand_group as default");

    if(!nhp.getParam("/right_wrist_link", right_wrist_link_))
        ROS_WARN("No right wrist link defined, using r_hand as default");
    if(!nhp.getParam("/right_palm_link",  right_palm_link_))
        ROS_WARN("No right palm link defined, using r_palm as default");
    if(!nhp.getParam("/right_hand_group", right_hand_group_))
        ROS_WARN("No right hand group defined, using r_hand_group as default");

    //LOADING ROBOT MODEL FOR JOINT NAMES
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    robot_model_ = robot_model_loader_->getModel();

    if (!nhp.getParam("/ot_library", this->ot_filename_))
        ROS_ERROR(" Did not find Object Template Library parameter /ot_library");
    else
        TemplateNodelet::loadObjectTemplateDatabaseXML(this->ot_filename_);

    if (!nhp.getParam("/r_hand_library", this->r_grasps_filename_))
        ROS_WARN(" Did not find Right Grasp Library parameter /r_hand_library, not using right grasps");
    else
        TemplateNodelet::loadGraspDatabaseXML(this->r_grasps_filename_, "right");

    if (!nhp.getParam("/l_hand_library", this->l_grasps_filename_))
        ROS_WARN(" Did not find Left Grasp Library parameter /l_hand_library, not using right grasps");
    else
        TemplateNodelet::loadGraspDatabaseXML(this->l_grasps_filename_, "left");

    // Load the robot specific ghost_poses database
    if (!nhp.getParam("/stand_poses_library", this->stand_filename_))
        ROS_WARN(" Did not find Stand Poses parameter /stand_poses_library, not using stand poses for robot");
    else
        TemplateNodelet::loadStandPosesDatabaseXML(this->stand_filename_);

    id_counter_ = 0;

    timer = nh_out.createTimer(ros::Duration(0.033), &TemplateNodelet::timerCallback, this);
}

void TemplateNodelet::timerCallback(const ros::TimerEvent& event)
{
    this->publishTemplateList();
}


void TemplateNodelet::addTemplateCb(const flor_ocs_msgs::OCSTemplateAdd::ConstPtr& msg)
{
    bool found = false;

    for (std::map<unsigned int,VigirObjectTemplate>::iterator it=object_template_map_.begin(); it!=object_template_map_.end(); ++it){
        if(it->second.path == (msg->template_path).substr(0, (msg->template_path).find_last_of(".")) ){ //removing file extension

            template_type_list_.push_back(it->second.type);	//Add the type of the template to be instantiated
            template_id_list_.push_back(id_counter_++);
            template_name_list_.push_back(msg->template_path);
            template_pose_list_.push_back(msg->pose);
            template_status_list_.push_back(0);  //Normal Status
            ROS_INFO("Added template to list with id: %d",(int)id_counter_);


            //ADD TEMPLATE TO PLANNING SCENE
            addCollisionObject(it->second.type,id_counter_-1,(msg->template_path).substr(0, (msg->template_path).find_last_of(".")),msg->pose.pose);
            found = true;
            break;
        }
    }

    if(!found)
        ROS_ERROR("Template %s not found in library, ignoring", (msg->template_path).substr(0, (msg->template_path).find_last_of(".")).c_str());

    this->publishTemplateList();
}

void TemplateNodelet::removeTemplateCb(const flor_ocs_msgs::OCSTemplateRemove::ConstPtr& msg)
{
    ROS_INFO("Removing template %d from list",(unsigned int)msg->template_id );
    int index = 0;
    for(; index < template_id_list_.size(); index++)
        if(template_id_list_[index] == msg->template_id)
            break;
    if(index < template_id_list_.size() && !template_id_list_.empty()
                                        && !template_type_list_.empty()
                                        && !template_name_list_.empty()
                                        && !template_pose_list_.empty()
                                        && !template_status_list_.empty())
    {
        //REMOVE TEMPLATE FROM THE PLANING SCENE
        ROS_INFO("Calling function to remove template %d attachment status: %d ",template_id_list_[index],template_status_list_[index]);
        removeCollisionObject(msg->template_id);

        template_id_list_.erase(template_id_list_.begin()+index);
        template_type_list_.erase(template_type_list_.begin()+index);	//Remove it
        template_name_list_.erase(template_name_list_.begin()+index);
        template_pose_list_.erase(template_pose_list_.begin()+index);
        template_status_list_.erase(template_status_list_.begin()+index);
        ROS_INFO("Removed! ");

        this->publishTemplateList();
    }else{
        if(index >= template_id_list_.size())
            ROS_ERROR("Tried to remove template which is not inside the instantiated template list");
        else
            ROS_ERROR("Tried to remove a template when template list is empty");
    }
}

void TemplateNodelet::updateTemplateCb(const flor_ocs_msgs::OCSTemplateUpdate::ConstPtr& msg)
{
    //ROS_INFO("Updating template %d",(unsigned int)msg->template_id);
    int index = 0;
    for(; index < template_id_list_.size(); index++)
        if(template_id_list_[index] == msg->template_id)
            break;
    if(index < template_id_list_.size())
    {
        //ROS_INFO("Updated!");
        template_pose_list_[index] = msg->pose;


        //UPDATE TEMPLATE POSE IN THE PLANNING SCENE
        moveCollisionObject(msg->template_id,msg->pose.pose);
    }
    this->publishTemplateList();
}

void TemplateNodelet::snapTemplateCb(const flor_grasp_msgs::TemplateSelection::ConstPtr& msg)
{
    //ROS_INFO("Updating template %d",(unsigned int)msg->template_id);
    int index = 0;
    for(; index < template_id_list_.size(); index++)
        if(template_id_list_[index] == msg->template_id.data)
            break;
    if(index < template_id_list_.size())
    {
        template_pose_list_[index] = last_attached_pose_;
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
    template_pose_list_[index] = msg->pose;
    publishTemplateList();
}

void TemplateNodelet::publishTemplateList()
{
    //std::cout << "timer" << std::endl;
    flor_ocs_msgs::OCSTemplateList cmd;

    cmd.template_id_list     = template_id_list_;
    cmd.template_list        = template_name_list_;
    cmd.template_type_list   = template_type_list_;
    cmd.template_status_list = template_status_list_;
    cmd.pose                 = template_pose_list_;

    // publish complete list of templates and poses
    template_list_pub_.publish( cmd );

    // publish to TF
    ros::Time now = ros::Time::now();

    std::vector<tf::StampedTransform> transforms;

    for(int i = 0; i < template_pose_list_.size(); i++)
    {
        tf::StampedTransform template_pose_to_tf;

        template_pose_to_tf.frame_id_ = "/world";
        std::stringstream ss;
        ss << "/template_tf_" << (unsigned int)template_id_list_[i];
        template_pose_to_tf.child_frame_id_ = ss.str();

        template_pose_to_tf.stamp_ = now;

        const geometry_msgs::Point& vec_l (template_pose_list_[i].pose.position);
        template_pose_to_tf.setOrigin(tf::Vector3(vec_l.x, vec_l.y, vec_l.z));

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(template_pose_list_[i].pose.orientation, orientation);

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
    if(hand_side == "left")
    {
        palm_link_  = left_palm_link_;
        wrist_link_ = left_wrist_link_;
        hand_group_ = left_hand_group_;
    }else{
        palm_link_  = right_palm_link_;
        wrist_link_ = right_wrist_link_;
        hand_group_ = right_hand_group_;
    }

    //Getting joints for hand from URDF robot description

    if(!robot_model_->hasLinkModel(palm_link_)){
        ROS_WARN("Hand model does not contain %s, not adding grasps",palm_link_.c_str());
        return;
    }

    robot_model::LinkTransformMap hand_palm_tf_map = robot_model_->getLinkModel(palm_link_)->getAssociatedFixedTransforms();
    ROS_INFO("Requested linktransform for %s",palm_link_.c_str());

    Eigen::Affine3d hand_palm_aff;
    bool found = false;

    for(robot_model::LinkTransformMap::iterator it = hand_palm_tf_map.begin(); it != hand_palm_tf_map.end(); ++it){
        ROS_INFO("Getting links in map: %s and comparing to: %s", it->first->getName().c_str(), wrist_link_.c_str());
        if(it->first->getName() == wrist_link_){
            ROS_INFO("Wrist %s found!!!",wrist_link_.c_str());
            hand_palm_aff = it->second;
            found = true;
            break;
        }
    }

    if(!found){
        ROS_WARN("Wrist %s NOT found!!!, setting to identity",wrist_link_.c_str());
        gp_T_lhand_.setIdentity();
        gp_T_rhand_.setIdentity();
    }else{
        if(hand_side=="left")
            tf::transformEigenToTF( hand_palm_aff,gp_T_lhand_);
        else
            tf::transformEigenToTF( hand_palm_aff,gp_T_rhand_);
    }

    if(robot_model_->hasJointModelGroup(hand_group_))
    {
        hand_joint_names_.clear();
        hand_joint_names_ = robot_model_->getJointModelGroup(hand_group_)->getActiveJointModelNames();
    }else{
        ROS_WARN("NO JOINTS FOUND FOR %s HAND",hand_side.c_str());
    }

    ROS_INFO("%s %s hand model gotten, #actuated joints: %ld ",hand_side.c_str(), robot_model_->getName().c_str(),hand_joint_names_.size() );

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
    for( pGrasps; pGrasps; pGrasps=pGrasps->NextSiblingElement("grasps")) //Iterates thorugh all template types
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
            grasp.grasp_pose.header.frame_id = wrist_link_;

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

                grasp.grasp_pose.header.frame_id    = wrist_link_;
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
                grasp.pre_grasp_approach.direction.header.frame_id = wrist_link_;

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
            grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

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
                    for( pFinger; pFinger; pFinger=pFinger->NextSiblingElement("finger"))   //Iterates thorugh all fingers for this particular pre posture
                    {
                        TiXmlElement* pJoint=pFinger->FirstChildElement( "joint" );       //Gets approaching vector
                        if(!pJoint){
                            ROS_WARN("Grasp ID: %s does not contain joints for finger %s",pID, pFinger->Attribute("idx"));
                        }else{
                            for( pJoint; pJoint; pJoint=pJoint->NextSiblingElement("joint"))   //Iterates thorugh all fingers for this particular pre posture
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
            grasp.grasp_posture.points[0].time_from_start = ros::Duration(0.5);

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
                    for( pFinger; pFinger; pFinger=pFinger->NextSiblingElement("finger"))   //Iterates thorugh all fingers for this particular posture
                    {
                        TiXmlElement* pJoint=pFinger->FirstChildElement( "joint" );       //Gets approaching vector
                        if(!pJoint){
                            ROS_WARN("Grasp ID: %s does not contain joints for finger %s",pID, pFinger->Attribute("idx"));
                        }else{
                            for( pJoint; pJoint; pJoint=pJoint->NextSiblingElement("joint"))   //Iterates thorugh all fingers for this particular posture
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
            grasp.post_grasp_retreat.direction.header.frame_id = "/world";
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

    ROS_INFO("%s Grasp Database loaded",hand_side.c_str());

}

void TemplateNodelet::loadStandPosesDatabaseXML(std::string& file_name){
    /*
     * Need to fill object_template_map_[type].stand_poses with the poses read from file
     *
     * ORDER IN FILE SHOULD MATCH ORDER IN READING
     * template type,
     * stand pose id,
     * stand pose relative to template (x,y,z,qw,qx,qy,qz),
    */
    ROS_INFO("Loading Stand Poses...");

    //Creating XML document from parameter server string of template library
    TiXmlDocument doc;
    doc.Parse((const char*)file_name.c_str(), 0, TIXML_ENCODING_UTF8);
    if (doc.ErrorId() != 0){
        ROS_ERROR("Could not read stand poses library file ");
        return;
    }

    TiXmlHandle hDoc(&doc);
    TiXmlElement* pElem;
    TiXmlHandle hRoot(0);

    pElem=hDoc.FirstChildElement().Element();
    // should always have a valid root but handle gracefully if it does
    if (!pElem){
        ROS_ERROR("File for stand poses library empty");
        return;
    }

    // save this for later
    hRoot=TiXmlHandle(pElem);
    ROS_INFO("Reading %s", pElem->Value());



    TiXmlElement* pTemplate=hRoot.FirstChild( "template" ).Element();
    for( pTemplate; pTemplate; pTemplate=pTemplate->NextSiblingElement("template")) //Iterates thorugh all template types
    {
        int template_type;
        const char *pName=pTemplate->Attribute("template_name");
        pTemplate->QueryIntAttribute("template_type", &template_type);
        if (pName) ROS_INFO("Reading %s type: %d",pName, template_type);

        TiXmlElement* pStandPose=pTemplate->FirstChildElement( "standpose" );
        for( pStandPose; pStandPose; pStandPose=pStandPose->NextSiblingElement("standpose" )) //Iterates thorugh all template types
        {
            vigir_object_template_msgs::StandPose current_pose;
            TiXmlElement* pPose=pStandPose->FirstChildElement("pose");
            if(!pPose){
                ROS_ERROR("Template ID: %d does not contain a  stand pose, skipping template",template_type);
                continue;
            }else{
                double qx,qy,qz,qw;
                std::string xyz = pPose->Attribute("xyz");
                std::istringstream iss(xyz);
                std::string word;
                std::vector<std::string> tokens;
                while ( iss >> word ) tokens.push_back( word );

                pPose->QueryDoubleAttribute("qx",&qx);
                pPose->QueryDoubleAttribute("qy",&qy);
                pPose->QueryDoubleAttribute("qz",&qz);
                pPose->QueryDoubleAttribute("qw",&qw);

                current_pose.id                      = std::atoi(pStandPose->Attribute("id"));
                current_pose.pose.header.frame_id    = "/world";
                current_pose.pose.header.stamp       = ros::Time::now();
                current_pose.pose.pose.position.x    = std::atof(tokens[0].c_str());
                current_pose.pose.pose.position.y    = std::atof(tokens[1].c_str());
                current_pose.pose.pose.position.z    = std::atof(tokens[2].c_str());
                current_pose.pose.pose.orientation.x = qx;
                current_pose.pose.pose.orientation.y = qy;
                current_pose.pose.pose.orientation.z = qz;
                current_pose.pose.pose.orientation.w = qw;

                if(object_template_map_.find(template_type) != object_template_map_.end())   //Template Type exists
                    object_template_map_[template_type].stand_poses.insert(std::pair<unsigned int,vigir_object_template_msgs::StandPose>(current_pose.id, current_pose));
            }
        }
    }
    for (std::map<unsigned int,VigirObjectTemplate>::iterator it=object_template_map_.begin(); it!=object_template_map_.end(); ++it)
        for (std::map<unsigned int,vigir_object_template_msgs::StandPose>::iterator it2=it->second.stand_poses.begin(); it2!=it->second.stand_poses.end(); ++it2)
            ROS_INFO("OT Map, inside ot: %d -> Stand pose id %d ", it->second.type, it2->second.id);

    ROS_INFO("Stand Poses Database loaded");
}

void TemplateNodelet::loadObjectTemplateDatabaseXML(std::string& file_name)
{
    //Creating XML document from parameter server string of template library
    TiXmlDocument doc;
    doc.Parse((const char*)file_name.c_str(), 0, TIXML_ENCODING_UTF8);
    if (doc.ErrorId() != 0){
        ROS_ERROR("Could not read object library file ");
        return;
    }

    TiXmlHandle hDoc(&doc);
    TiXmlElement* pElem;
    TiXmlHandle hRoot(0);

    pElem=hDoc.FirstChildElement().Element();
    // should always have a valid root but handle gracefully if it does
    if (!pElem){
        ROS_ERROR("File for template library empty");
        return;
    }

    // save this for later
    hRoot=TiXmlHandle(pElem);
    ROS_INFO("Reading %s", pElem->Value());

    TiXmlElement* pTemplate=hRoot.FirstChild( "template" ).Element();
    int count=0;
    for( pTemplate; pTemplate; pTemplate=pTemplate->NextSiblingElement( "template"), count++) //Iterates thorugh all template types
    {
        //Getting type and name
        VigirObjectTemplate object_template;
        const char *pName=pTemplate->Attribute("template_name");
        const char *pGroup=pTemplate->Attribute("group");
        int template_type;
        pTemplate->QueryIntAttribute("template_type", &template_type);
        if (pName) ROS_INFO("Reading %s type: %d",pName, template_type);

        object_template.name = pName;
        object_template.type = template_type;
        object_template.id   = count;
        object_template.path = std::string(pGroup) + "/" + std::string(pName);

//        //Getting Mesh Path
//        TiXmlElement* pMesh=pTemplate->FirstChildElement( "visual" )->FirstChildElement("geometry")->FirstChildElement("mesh");
//        if(!pMesh){
//            ROS_ERROR("Template ID: %d does not contain a mesh path, skipping template",template_type);
//            continue;
//        }else{
//            object_template.path = pMesh->Attribute("filename");
//        }


        //Getting Mass
        TiXmlElement* pMass=pTemplate->FirstChildElement( "inertial" )->FirstChildElement("mass");
        if(!pMass){
            ROS_WARN("Template ID: %d does not contain a mass value, setting to 0",template_type);
            object_template.mass = 0;
        }else{
            object_template.mass = std::atof(pMass->Attribute("value"));
        }

        //Getting CoM
        TiXmlElement* pCoM=pTemplate->FirstChildElement( "inertial" )->FirstChildElement("origin");
        if(!pCoM){
            ROS_WARN("Template ID: %d does not contain a mass value, setting to 0",template_type);
            object_template.com.x = object_template.com.y = object_template.com.z = 0;
        }else{
            geometry_msgs::Point com;
            std::string xyz = pCoM->Attribute("xyz");
            std::istringstream iss(xyz);
            std::string word;
            std::vector<std::string> tokens;
            while ( iss >> word ) tokens.push_back( word );

            com.x = std::atof(tokens[0].c_str());
            com.y = std::atof(tokens[1].c_str());
            com.z = std::atof(tokens[2].c_str());

            object_template.com = com;
        }

        //Getting Inertia
        TiXmlElement* pInertia=pTemplate->FirstChildElement( "inertial" )->FirstChildElement("inertia");
        if(!pInertia){
            ROS_WARN("Template ID: %d does not contain an inertia tensor",template_type);
        }else{
            //object_template.inertia.ixx = std::atof(pInertia->Attribute("ixx"));
            //object_template.inertia.ixy = std::atof(pInertia->Attribute("ixy"));
            //object_template.inertia.ixz = std::atof(pInertia->Attribute("ixz"));
            //object_template.inertia.iyy = std::atof(pInertia->Attribute("iyy"));
            //object_template.inertia.iyz = std::atof(pInertia->Attribute("iyz"));
            //object_template.inertia.izz = std::atof(pInertia->Attribute("izz"));
        }

        //Getting bounding box
        TiXmlElement* pBoundingBox=pTemplate->FirstChildElement( "visual" )->FirstChildElement("geometry")->FirstChildElement("boundingbox");
        if(!pBoundingBox){
            ROS_WARN("Template ID: %d does not contain a  bounding box, setting to zeros",template_type);
            object_template.b_max.x = object_template.b_max.y = object_template.b_max.z = 0;
            object_template.b_min.x = object_template.b_min.y = object_template.b_min.z = 0;
        }else{
            geometry_msgs::Point b_max;
            geometry_msgs::Point b_min;
            std::string xyz = pBoundingBox->Attribute("min");
            std::istringstream iss(xyz);
            std::string word;
            std::vector<std::string> tokens;
            while ( iss >> word ) tokens.push_back( word );

            b_min.x    = std::atof(tokens[0].c_str());
            b_min.y    = std::atof(tokens[1].c_str());
            b_min.z    = std::atof(tokens[2].c_str());

            xyz = pBoundingBox->Attribute("max");
            std::istringstream iss2(xyz);
            tokens.clear();
            while ( iss2 >> word ) tokens.push_back( word );

            b_max.x    = std::atof(tokens[0].c_str());
            b_max.y    = std::atof(tokens[1].c_str());
            b_max.z    = std::atof(tokens[2].c_str());

            object_template.b_max = b_max;
            object_template.b_min = b_min;
        }

        //Getting usabilities
        TiXmlElement* pUsability=pTemplate->FirstChildElement( "usability" );
        for( pUsability; pUsability; pUsability=pUsability->NextSiblingElement("usability")) //Iterates thorugh all usabilities
        {
            vigir_object_template_msgs::Usability usability;
            TiXmlElement* pPose=pUsability->FirstChildElement("pose");
            if(!pPose){
                ROS_ERROR("Template ID: %d does not contain a  pose in usability id: %s, skipping template",template_type,pUsability->Attribute("id"));
                continue;
            }else{
                double qx,qy,qz,qw;
                std::string xyz = pPose->Attribute("xyz");
                std::istringstream iss(xyz);
                std::string word;
                std::vector<std::string> tokens;
                while ( iss >> word ) tokens.push_back( word );

                pPose->QueryDoubleAttribute("qx",&qx);
                pPose->QueryDoubleAttribute("qy",&qy);
                pPose->QueryDoubleAttribute("qz",&qz);
                pPose->QueryDoubleAttribute("qw",&qw);

                usability.id                      = std::atoi(pUsability->Attribute("id"));
                usability.pose.header.frame_id    = "/world";
                usability.pose.header.stamp       = ros::Time::now();
                usability.pose.pose.position.x    = std::atof(tokens[0].c_str());
                usability.pose.pose.position.y    = std::atof(tokens[1].c_str());
                usability.pose.pose.position.z    = std::atof(tokens[2].c_str());
                usability.pose.pose.orientation.x = qx;
                usability.pose.pose.orientation.y = qy;
                usability.pose.pose.orientation.z = qz;
                usability.pose.pose.orientation.w = qw;

                object_template.usabilities.insert(std::pair<unsigned int,vigir_object_template_msgs::Usability>(usability.id, usability));
            }
        }

        //Getting Affordances
        TiXmlElement* pAffordance=pTemplate->FirstChildElement( "affordance" );
        unsigned int aff_idx = 100;
        for( pAffordance; pAffordance; pAffordance=pAffordance->NextSiblingElement("affordance")) //Iterates thorugh all affordances
        {
            vigir_object_template_msgs::Affordance affordance;
            if(pAffordance->Attribute("id")){
                affordance.id = std::atoi(pAffordance->Attribute("id"));
            }else{
                ROS_WARN("Affordance ID not found, setting to %d",aff_idx);
                affordance.id = aff_idx;
                aff_idx++;
            }

            TiXmlElement* pPose=pAffordance->FirstChildElement("pose");
            if(!pPose){
                ROS_ERROR("Template ID: %d does not contain a  pose in affordance id: %s, skipping template",template_type,pAffordance->Attribute("id"));
                continue;
            }else{
                ROS_INFO("Getting poses for affordance %d", affordance.id);
                for(pPose; pPose; pPose=pPose->NextSiblingElement("pose")){

                    double qx,qy,qz,qw;
                    std::string xyz = pPose->Attribute("xyz");
                    std::istringstream iss(xyz);
                    std::string word;
                    std::vector<std::string> tokens;
                    while ( iss >> word ) tokens.push_back( word );

                    pPose->QueryDoubleAttribute("qx",&qx);
                    pPose->QueryDoubleAttribute("qy",&qy);
                    pPose->QueryDoubleAttribute("qz",&qz);
                    pPose->QueryDoubleAttribute("qw",&qw);

                    geometry_msgs::PoseStamped waypoint;

                    waypoint.header.frame_id    = "/world";
                    waypoint.header.stamp       = ros::Time::now();
                    waypoint.pose.position.x    = std::atof(tokens[0].c_str());
                    waypoint.pose.position.y    = std::atof(tokens[1].c_str());
                    waypoint.pose.position.z    = std::atof(tokens[2].c_str());
                    waypoint.pose.orientation.x = qx;
                    waypoint.pose.orientation.y = qy;
                    waypoint.pose.orientation.z = qz;
                    waypoint.pose.orientation.w = qw;
                    affordance.waypoints.push_back(waypoint);
                    ROS_INFO("Getting %d waypoints", (int)affordance.waypoints.size());
                }
                ROS_INFO("Finished getting poses");


                if(pAffordance->Attribute("name"))
                    affordance.name = pAffordance->Attribute("name");
                else{
                    ROS_WARN("Affordance ID: %d has no name attribute, setting to aff_%d", affordance.id, affordance.id);
                    affordance.name = "aff_" + boost::to_string(affordance.id);
                }

                if(pAffordance->Attribute("type"))
                    affordance.type = pAffordance->Attribute("type");
                else{
                    ROS_WARN("Affordance ID: %d has no type attribute, setting to no_type", affordance.id);
                    affordance.type = "no_type";
                }

                if(pAffordance->Attribute("axis"))
                    affordance.axis = pAffordance->Attribute("axis");
                else{
                    ROS_WARN("Affordance ID: %d has no axis attribute, setting to no_axis", affordance.id);
                    affordance.axis = "no_axis";
                }

                if(pAffordance->Attribute("displacement"))
                    affordance.displacement = std::atof(pAffordance->Attribute("displacement"));
                else{
                    ROS_WARN("Affordance ID: %d has no displacement attribute, setting to zero", affordance.id);
                    affordance.displacement = 0.0;
                }

                if(pAffordance->Attribute("keeporientation")){
                    if(std::string(pAffordance->Attribute("keeporientation")) == std::string("true"))
                        affordance.keep_orientation = true;
                    else
                        affordance.keep_orientation = false;

                }else{
                    ROS_WARN("Affordance ID: %d has no keeporientation attribute, setting to true", affordance.id);
                    affordance.keep_orientation     = true;
                }

                object_template.affordances.insert(std::pair<unsigned int,vigir_object_template_msgs::Affordance>(affordance.id, affordance));
            }
        }

        object_template_map_.insert(std::pair<unsigned int,VigirObjectTemplate>(template_type,object_template));
        ROS_INFO(" Inserting Object template type: %d with id: %d, name %s and mesh path: %s, aff.name: %s, aff.displacement: %f", object_template_map_[template_type].type
                                                                                             , object_template_map_[template_type].id
                                                                                             , object_template_map_[template_type].name.c_str()
                                                                                             , object_template_map_[template_type].path.c_str()
                                                                                             , object_template_map_[template_type].affordances[1].name.c_str()
                                                                                             , object_template_map_[template_type].affordances[1].displacement);
    }

    ROS_INFO("OT Database loaded");
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
    unsigned int template_type;
	for(; index < template_id_list_.size(); index++) {
        if(template_id_list_[index] == req.template_id){
            template_type = template_type_list_[index];
            ROS_INFO("Template ID found in server, index: %d, list: %d, requested: %d",index, template_id_list_[index], req.template_id);
            break;
		}
    }
	
    if (index >= template_id_list_.size()){
        ROS_ERROR("Service requested template ID %d when no such template has been instantiated. Callback returning false.",req.template_id);
		return false;
    }

    if(template_pose_list_[index].header.frame_id != "/world"){
        ROS_ERROR("Template not in /world frame, detach from robot!");
        return false;
    }

    res.template_state_information.template_type = template_type;
    res.template_state_information.template_id   = req.template_id;
    res.template_state_information.type_name 	 = object_template_map_[template_type].name;
    res.template_state_information.pose 		 = template_pose_list_[index];

    res.template_type_information.type_name      = object_template_map_[template_type].name;
    res.template_type_information.mass           = object_template_map_[template_type].mass;
    res.template_type_information.center_of_mass = object_template_map_[template_type].com;
    res.template_type_information.b_max          = object_template_map_[template_type].b_max;
    res.template_type_information.b_min          = object_template_map_[template_type].b_min;
	
	//Transfer all known grasps to response
    for (std::map<unsigned int,moveit_msgs::Grasp>::iterator it =  object_template_map_[template_type].grasps.begin();
                                                             it != object_template_map_[template_type].grasps.end();
                                                             ++it) {
        //Transform to world coordinate frame
        moveit_msgs::Grasp grasp = it->second;
        grasp.grasp_posture.header.frame_id                = "/world";
        grasp.post_grasp_retreat.direction.header.frame_id = "/world";
        grasp.post_place_retreat.direction.header.frame_id = "/world";
        grasp.pre_grasp_approach.direction.header.frame_id = "/world";
        grasp.pre_grasp_posture.header.frame_id            = "/world";
        if(std::atoi(grasp.id.c_str()) >= 1000 && (req.hand_side == req.LEFT_HAND || req.hand_side == req.BOTH_HANDS)){
            staticTransform(grasp.grasp_pose.pose,gp_T_lhand_);
            worldPoseTransform(template_pose_list_[index],grasp.grasp_pose.pose,grasp.grasp_pose);
            res.template_type_information.grasps.push_back(grasp);
        }
        if(std::atoi(grasp.id.c_str()) < 1000 && (req.hand_side == req.RIGHT_HAND || req.hand_side == req.BOTH_HANDS)){
            staticTransform(grasp.grasp_pose.pose,gp_T_rhand_);
            worldPoseTransform(template_pose_list_[index],grasp.grasp_pose.pose,grasp.grasp_pose);
            res.template_type_information.grasps.push_back(grasp);
        }

    }

    //Transfer all known stand poses to response
    for (std::map<unsigned int,vigir_object_template_msgs::StandPose>::iterator it =  object_template_map_[template_type].stand_poses.begin();
                                                                                it != object_template_map_[template_type].stand_poses.end();
                                                                                ++it) {
        //Transform to world coordinate frame
        vigir_object_template_msgs::StandPose stand_pose = it->second;
        if(template_pose_list_[index].header.frame_id == "/world")
            worldPoseTransform(template_pose_list_[index],stand_pose.pose.pose,stand_pose.pose);
        else{
            ROS_ERROR("Template not in /world frame, detach from robot!");
            return false;
        }
        res.template_type_information.stand_poses.push_back(stand_pose);
    }

    //Transfer all known usabilities to response
    for (std::map<unsigned int,vigir_object_template_msgs::Usability>::iterator it =  object_template_map_[template_type].usabilities.begin();
                                                                                it != object_template_map_[template_type].usabilities.end();
                                                                                ++it) {
        vigir_object_template_msgs::Usability usability = it->second;
        if(template_pose_list_[index].header.frame_id == "/world")
            worldPoseTransform(template_pose_list_[index],usability.pose.pose,usability.pose);
        else{
            ROS_ERROR("Template not in /world frame, detach from robot!");
            return false;
        }
        res.template_type_information.usabilities.push_back(usability);
    }

    //Transfer all known usabilities to response
    for (std::map<unsigned int,vigir_object_template_msgs::Affordance>::iterator it =  object_template_map_[template_type].affordances.begin();
                                                                                 it != object_template_map_[template_type].affordances.end();
                                                                                 ++it) {
        vigir_object_template_msgs::Affordance affordance = it->second;
        if(template_pose_list_[index].header.frame_id == "/world")
            for(int waypoint=0; waypoint < affordance.waypoints.size(); waypoint++)
                worldPoseTransform(template_pose_list_[index],affordance.waypoints[waypoint].pose,affordance.waypoints[waypoint]);
        else{
            ROS_ERROR("Template not in /world frame, detach from robot!");
            return false;
        }
        res.template_type_information.affordances.push_back(affordance);
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
    res.template_type_information.geometry_marker.mesh_resource   = object_template_map_[template_type].path;
    res.template_type_information.geometry_marker.pose            = template_pose_list_[index].pose;

  return true;
}

bool TemplateNodelet::graspInfoSrv(vigir_object_template_msgs::GetGraspInfo::Request& req,
                                   vigir_object_template_msgs::GetGraspInfo::Response& res)
{
    /*Fill in the blanks of the response "res"
     * with the info of the template id in the request "req"
    */
    if(object_template_map_.size() > 0){
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
    return false;
}

bool TemplateNodelet::instantiatedGraspInfoSrv(vigir_object_template_msgs::GetInstantiatedGraspInfo::Request& req,
                                               vigir_object_template_msgs::GetInstantiatedGraspInfo::Response& res)
{
    /*Fill in the blanks of the response "res"
     * with the info of the template id in the request "req"
    */
    unsigned int index = 0;
    unsigned int template_type;
    for(; index < template_id_list_.size(); index++) {
        if(template_id_list_[index] == req.template_id){
            template_type = template_type_list_[index];
            ROS_INFO("Template ID found in server, index: %d, list: %d, requested: %d",index, template_id_list_[index], req.template_id);
            break;
        }
    }

    if (index >= template_id_list_.size()){
        ROS_ERROR("Service requested template ID %d when no such template has been instantiated. Callback returning false.",req.template_id);
        return false;
    }
    //Transfer all known grasps to response
    for (std::map<unsigned int,moveit_msgs::Grasp>::iterator it =  object_template_map_[template_type].grasps.begin();
                                                             it != object_template_map_[template_type].grasps.end();
                                                             ++it) {
        //Transform to world coordinate frame
        moveit_msgs::Grasp grasp = it->second;
        moveit_msgs::Grasp pre_grasp = it->second;
        grasp.grasp_posture.header.frame_id                = "/world";
        grasp.post_grasp_retreat.direction.header.frame_id = "/world";
        grasp.post_place_retreat.direction.header.frame_id = "/world";
        grasp.pre_grasp_approach.direction.header.frame_id = "/world";
        grasp.pre_grasp_posture.header.frame_id            = "/world";
        pre_grasp.grasp_posture.header.frame_id                = "/world";
        pre_grasp.post_grasp_retreat.direction.header.frame_id = "/world";
        pre_grasp.post_place_retreat.direction.header.frame_id = "/world";
        pre_grasp.pre_grasp_approach.direction.header.frame_id = "/world";
        pre_grasp.pre_grasp_posture.header.frame_id            = "/world";
        if(std::atoi(grasp.id.c_str()) >= 1000 && (req.hand_side == req.LEFT_HAND || req.hand_side == req.BOTH_HANDS)){
            staticTransform(grasp.grasp_pose.pose,gp_T_lhand_);
            gripperTranslationToPreGraspPose(pre_grasp.grasp_pose.pose,pre_grasp.pre_grasp_approach);
            staticTransform(pre_grasp.grasp_pose.pose,gp_T_lhand_);
            worldPoseTransform(template_pose_list_[index],grasp.grasp_pose.pose,grasp.grasp_pose);
            worldPoseTransform(template_pose_list_[index],pre_grasp.grasp_pose.pose,pre_grasp.grasp_pose);
            res.grasp_information.grasps.push_back(grasp);
            res.pre_grasp_information.grasps.push_back(pre_grasp);
        }
        if(std::atoi(grasp.id.c_str()) < 1000 && (req.hand_side == req.RIGHT_HAND || req.hand_side == req.BOTH_HANDS)){
            staticTransform(grasp.grasp_pose.pose,gp_T_rhand_);
            gripperTranslationToPreGraspPose(pre_grasp.grasp_pose.pose,pre_grasp.pre_grasp_approach);
            staticTransform(pre_grasp.grasp_pose.pose,gp_T_rhand_);
            worldPoseTransform(template_pose_list_[index],grasp.grasp_pose.pose,grasp.grasp_pose);
            worldPoseTransform(template_pose_list_[index],pre_grasp.grasp_pose.pose,pre_grasp.grasp_pose);
            res.grasp_information.grasps.push_back(grasp);
            res.pre_grasp_information.grasps.push_back(pre_grasp);
        }

    }

    //Transfer all known stand poses to response
    for (std::map<unsigned int,vigir_object_template_msgs::StandPose>::iterator it =  object_template_map_[template_type].stand_poses.begin();
                                                                                it != object_template_map_[template_type].stand_poses.end();
                                                                                ++it) {
        //Transform to world coordinate frame
        vigir_object_template_msgs::StandPose stand_pose = it->second;
        worldPoseTransform(template_pose_list_[index],stand_pose.pose.pose,stand_pose.pose);
        res.grasp_information.stand_poses.push_back(stand_pose);
        res.pre_grasp_information.stand_poses.push_back(stand_pose);
    }
    return true;
}

bool TemplateNodelet::attachObjectTemplateSrv(vigir_object_template_msgs::SetAttachedObjectTemplate::Request& req,
                                              vigir_object_template_msgs::SetAttachedObjectTemplate::Response& res)
{

    /* First, define the REMOVE object message*/
    moveit_msgs::CollisionObject remove_object;
    remove_object.id              = boost::to_string(req.template_id);
    remove_object.header.frame_id = "/world";
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
            mesh_name = (template_name_list_[index]).substr(0, (template_name_list_[index]).find_last_of("."));
            template_pose      = template_pose_list_[index];
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

    tf::Transform world_T_wrist;
    tf::Transform world_T_template;
    tf::Transform target_pose;
    world_T_wrist.setRotation(tf::Quaternion(req.pose.pose.orientation.x,req.pose.pose.orientation.y,req.pose.pose.orientation.z,req.pose.pose.orientation.w));
    world_T_wrist.setOrigin(tf::Vector3(req.pose.pose.position.x,req.pose.pose.position.y,req.pose.pose.position.z) );
    world_T_template.setRotation(tf::Quaternion(template_pose.pose.orientation.x,template_pose.pose.orientation.y,template_pose.pose.orientation.z,template_pose.pose.orientation.w));
    world_T_template.setOrigin(tf::Vector3(template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z) );

    target_pose = world_T_wrist.inverse() * world_T_template;

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

    template_pose_list_[index].header.frame_id = req.pose.header.frame_id; //Attaches the OCS template to the robot hand
    template_pose_list_[index].pose            = pose;
    template_status_list_[index]               = 1; //Attached to robot

    last_attached_pose_ = template_pose_list_[index];

    // Note that attaching an object to the robot requires
    // the corresponding operation to be specified as an ADD operation
    attached_object.object.operation = attached_object.object.ADD;

    if(req.pose.header.frame_id == right_wrist_link_){
        hand_link_names_ = robot_model_->getJointModelGroup(right_hand_group_)->getLinkModelNames();
        hand_link_names_.push_back(right_palm_link_);
    }else{
        hand_link_names_ = robot_model_->getJointModelGroup(left_hand_group_)->getLinkModelNames();
        hand_link_names_.push_back(left_palm_link_);
    }


    for(int i = 0; i < hand_link_names_.size(); i++){
        ROS_INFO("Link %d: %s",i,hand_link_names_[i].c_str());
        attached_object.touch_links.push_back(hand_link_names_[i]);
    }

    ROS_INFO("Attaching the object to the %s link, template %d status: %d", req.pose.header.frame_id.c_str(), template_id_list_[index], template_status_list_[index]);
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
            mesh_name = (template_name_list_[index]).substr(0, (template_name_list_[index]).find_last_of("."));
            template_pose      = template_pose_list_[index];
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

    geometry_msgs::Pose pose;

    ROS_INFO("Template is in %s frame", template_pose.header.frame_id.c_str());

    if(template_pose.header.frame_id == "/world"){
        tf::Transform hand_T_template;
        tf::Transform world_T_hand;
        tf::Transform world_T_template;
        world_T_hand.setRotation(tf::Quaternion(req.pose.pose.orientation.x,req.pose.pose.orientation.y,req.pose.pose.orientation.z,req.pose.pose.orientation.w));
        world_T_hand.setOrigin(tf::Vector3(req.pose.pose.position.x,req.pose.pose.position.y,req.pose.pose.position.z) );
        world_T_template.setRotation(tf::Quaternion(template_pose.pose.orientation.x,template_pose.pose.orientation.y,template_pose.pose.orientation.z,template_pose.pose.orientation.w));
        world_T_template.setOrigin(tf::Vector3(template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z) );

        hand_T_template = world_T_hand.inverse() * world_T_template;

        pose.orientation.x = hand_T_template.getRotation().getX();
        pose.orientation.y = hand_T_template.getRotation().getY();
        pose.orientation.z = hand_T_template.getRotation().getZ();
        pose.orientation.w = hand_T_template.getRotation().getW();
        pose.position.x    = hand_T_template.getOrigin().getX();
        pose.position.y    = hand_T_template.getOrigin().getY();
        pose.position.z    = hand_T_template.getOrigin().getZ();
    }else{
        pose = template_pose.pose;
    }

    tmp_attached_object.object.meshes.push_back(mesh_);
    tmp_attached_object.object.mesh_poses.push_back(pose);

    template_pose_list_[index].header.frame_id = req.pose.header.frame_id; //Attaches the OCS template to the robot hand
    template_pose_list_[index].pose            = pose;
    template_status_list_[index]               = 1; //Attached to robot

    last_attached_pose_ = template_pose_list_[index];

    // Note that attaching an object to the robot requires
    // the corresponding operation to be specified as an ADD operation
    tmp_attached_object.object.operation = tmp_attached_object.object.ADD;

    if(req.pose.header.frame_id == right_wrist_link_){
        hand_link_names_ = robot_model_->getJointModelGroup(right_hand_group_)->getLinkModelNames();
        hand_link_names_.push_back(right_palm_link_);
    }else{
        hand_link_names_ = robot_model_->getJointModelGroup(left_hand_group_)->getLinkModelNames();
        hand_link_names_.push_back(left_palm_link_);
    }

    for(int i = 0; i < hand_link_names_.size(); i++){
        ROS_INFO("Link %d: %s",i,hand_link_names_[i].c_str());
        tmp_attached_object.touch_links.push_back(hand_link_names_[i]);
    }

    ros::Duration(0.5).sleep();

    ROS_INFO("Removing collision object :%s started",tmp_attached_object.object.id.c_str());
    /* First, define the REMOVE object message*/
    moveit_msgs::CollisionObject remove_object;
    remove_object.id = tmp_attached_object.object.id;
    remove_object.header.frame_id = "/world";
    remove_object.operation = remove_object.REMOVE;

    ROS_INFO("Collision object :%s removed",remove_object.id.c_str());

    co_pub_.publish(remove_object);

    ROS_INFO("Stitching the object %s",tmp_attached_object.object.id.c_str());
    aco_pub_.publish(tmp_attached_object);
    return true;

}

bool TemplateNodelet::detachObjectTemplateSrv(vigir_object_template_msgs::SetAttachedObjectTemplate::Request& req,
                                              vigir_object_template_msgs::SetAttachedObjectTemplate::Response& res)
{
    /* First, define the DETACH object message*/
    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = boost::to_string(req.template_id);
    detach_object.object.operation = detach_object.object.REMOVE;

    geometry_msgs::PoseStamped template_pose;

    unsigned int index = 0;
    for(; index < template_id_list_.size(); index++) {
        if(template_id_list_[index] == req.template_id){
            template_pose                              = template_pose_list_[index];
            break;
        }
    }

    if (index >= template_id_list_.size()){
        //ROS_ERROR_STREAM("Service requested template id " << req.template_type.data << " when no such id has been instantiated. Callback returning false.");
        ROS_ERROR("Service requested template id %d when no such id has been instantiated. Callback returning false.",req.template_id);
        return false;
    }

    if(template_pose.header.frame_id != "/world"){

        tf::Transform world_T_wrist;
        tf::Transform wrist_T_template;
        tf::Transform world_T_template;
        world_T_wrist.setRotation(tf::Quaternion(req.pose.pose.orientation.x,req.pose.pose.orientation.y,req.pose.pose.orientation.z,req.pose.pose.orientation.w));
        world_T_wrist.setOrigin(tf::Vector3(req.pose.pose.position.x,req.pose.pose.position.y,req.pose.pose.position.z) );
        wrist_T_template.setRotation(tf::Quaternion(template_pose.pose.orientation.x,template_pose.pose.orientation.y,template_pose.pose.orientation.z,template_pose.pose.orientation.w));
        wrist_T_template.setOrigin(tf::Vector3(template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z) );

        world_T_template = world_T_wrist * wrist_T_template;

        geometry_msgs::Pose pose;
        pose.orientation.x = world_T_template.getRotation().getX();
        pose.orientation.y = world_T_template.getRotation().getY();
        pose.orientation.z = world_T_template.getRotation().getZ();
        pose.orientation.w = world_T_template.getRotation().getW();
        pose.position.x    = world_T_template.getOrigin().getX();
        pose.position.y    = world_T_template.getOrigin().getY();
        pose.position.z    = world_T_template.getOrigin().getZ();

        template_pose_list_[index].header.frame_id = "/world"; //Attaches the OCS template to world
        template_pose_list_[index].pose            = pose;     //Pose of the template in world
    }
    template_status_list_[index]               = 0;        //Detached from robot

    ROS_INFO("Dettaching the object %s",detach_object.object.id.c_str());
    aco_pub_.publish(detach_object);
}

int TemplateNodelet::worldPoseTransform(const geometry_msgs::PoseStamped& template_pose,const geometry_msgs::Pose& input_pose, geometry_msgs::PoseStamped& target_pose)
{
    tf::Transform template_T_wrist;
    tf::Transform world_T_template;
    tf::Transform world_T_wrist;

    world_T_template.setRotation(tf::Quaternion(template_pose.pose.orientation.x,template_pose.pose.orientation.y,template_pose.pose.orientation.z,template_pose.pose.orientation.w));
    world_T_template.setOrigin(tf::Vector3(template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z) );
    template_T_wrist.setRotation(tf::Quaternion(input_pose.orientation.x,input_pose.orientation.y,input_pose.orientation.z,input_pose.orientation.w));
    template_T_wrist.setOrigin(tf::Vector3(input_pose.position.x,input_pose.position.y,input_pose.position.z) );

    world_T_wrist = world_T_template * template_T_wrist;

    tf::Quaternion tg_quat;
    tf::Vector3    tg_vector;
    tg_quat   = world_T_wrist.getRotation();
    tg_vector = world_T_wrist.getOrigin();

    target_pose.pose.orientation.w = tg_quat.getW();
    target_pose.pose.orientation.x = tg_quat.getX();
    target_pose.pose.orientation.y = tg_quat.getY();
    target_pose.pose.orientation.z = tg_quat.getZ();

    target_pose.pose.position.x = tg_vector.getX();
    target_pose.pose.position.y = tg_vector.getY();
    target_pose.pose.position.z = tg_vector.getZ();

    target_pose.header.frame_id = "/world";
    target_pose.header.stamp    = template_pose.header.stamp;
    return 0;
}

int TemplateNodelet::poseTransform(geometry_msgs::Pose& first_pose, geometry_msgs::Pose& second_pose)
{
    tf::Transform output_transform;
    tf::Transform first_transform;
    tf::Transform second_transform;

    first_transform.setRotation(tf::Quaternion(first_pose.orientation.x,first_pose.orientation.y,first_pose.orientation.z,first_pose.orientation.w));
    first_transform.setOrigin(tf::Vector3(first_pose.position.x,first_pose.position.y,first_pose.position.z) );

    second_transform.setRotation(tf::Quaternion(second_pose.orientation.x,second_pose.orientation.y,second_pose.orientation.z,second_pose.orientation.w));
    second_transform.setOrigin(tf::Vector3(second_pose.position.x,second_pose.position.y,second_pose.position.z) );

    output_transform = first_transform * second_transform;

    tf::Quaternion output_quat;
    tf::Vector3    output_vector;
    output_quat   = output_transform.getRotation();
    output_vector = output_transform.getOrigin();

    first_pose.position.x    = output_vector.getX();
    first_pose.position.y    = output_vector.getY();
    first_pose.position.z    = output_vector.getZ();
    first_pose.orientation.x = output_quat.getX();
    first_pose.orientation.y = output_quat.getY();
    first_pose.orientation.z = output_quat.getZ();
    first_pose.orientation.w = output_quat.getW();
    return 0;
}

// transform endeffort to palm pose used by GraspIt
int TemplateNodelet::staticTransform(geometry_msgs::Pose& palm_pose, tf::Transform gp_T_hand)
{
    tf::Transform o_T_hand;    //describes hand in object's frame
    tf::Transform o_T_pg;       //describes palm_from_graspit in object's frame

    o_T_pg.setRotation(tf::Quaternion(palm_pose.orientation.x,palm_pose.orientation.y,palm_pose.orientation.z,palm_pose.orientation.w));
    o_T_pg.setOrigin(tf::Vector3(palm_pose.position.x,palm_pose.position.y,palm_pose.position.z) );

    o_T_hand = o_T_pg * gp_T_hand;

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


void TemplateNodelet::addCollisionObject(int type, int index, std::string mesh_name, geometry_msgs::Pose pose){
    //Add collision object with template pose and bounding box

    ROS_INFO("Add collision template started... ");
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = boost::to_string((unsigned int)index);
    collision_object.header.frame_id = "/world";

    std::string path = ros::package::getPath("vigir_template_library") + "/object_templates/" + mesh_name + ".ply";

    std::ifstream infile(path.c_str());
    if(infile.good()){
        std::string mesh_path = "package://vigir_template_library/object_templates/"+mesh_name + ".ply";
        ROS_INFO("1 mesh_path: %s", mesh_path.c_str());

        shapes::Mesh* shape = shapes::createMeshFromResource(mesh_path);
        shape_msgs::Mesh mesh_;
        shapes::ShapeMsg mesh_msg = mesh_;
        shapes::constructMsgFromShape(shape,mesh_msg);
        mesh_ = boost::get<shape_msgs::Mesh>(mesh_msg);

        collision_object.meshes.push_back(mesh_);
        collision_object.mesh_poses.push_back(pose);

//        collision_object.primitives.resize(1);
//        collision_object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
//        collision_object.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
//        float x_size = object_template_map_[type].b_max.x - object_template_map_[type].b_min.x;
//        float y_size = object_template_map_[type].b_max.y - object_template_map_[type].b_min.y;
//        float z_size = object_template_map_[type].b_max.z - object_template_map_[type].b_min.z;
//        collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = x_size;
//        collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = y_size;
//        collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = z_size;
//        collision_object.primitive_poses.push_back(pose);


        collision_object.operation = collision_object.ADD;
        ROS_INFO("Adding the object: %s to the environment",collision_object.id.c_str());
        co_pub_.publish(collision_object);
    }else{
        ROS_ERROR("Mesh file: %s doesn't exists",path.c_str());
    }
}

void TemplateNodelet::moveCollisionObject(int template_id, geometry_msgs::Pose pose){
    //Add collision object with template pose and bounding box

    //ROS_INFO("Move collision template started... ");

    unsigned int idx = 0;
    for(; idx < template_id_list_.size(); idx++) {
        if(template_id_list_[idx] == template_id){
            break;
        }
    }
    if(idx >= template_id_list_.size()){
        ROS_ERROR("Collision Object %d not found!", template_id);
    }else{
        if(template_status_list_[idx] == 0){
            moveit_msgs::CollisionObject collision_object;
            collision_object.id              = boost::to_string((unsigned int)template_id);
            collision_object.header.frame_id = "/world";
            collision_object.mesh_poses.push_back(pose);
            collision_object.operation       = collision_object.MOVE;
            //ROS_INFO("Moving the object in the environment");
            co_pub_.publish(collision_object);
        }else
            if(template_status_list_[idx] == 1)
                ROS_INFO("Object Template %d attached to robot, cannot move!",template_id);
            else
                ROS_ERROR("Something is not OK with template %d status! status: %d, should be 1 or 0", template_id_list_[idx], template_status_list_[idx]);
    }

}

void TemplateNodelet::removeCollisionObject(int template_id){
    //Add collision object with template pose and bounding box

    unsigned int idx = 0;
    for(; idx < template_id_list_.size(); idx++) {
        if(template_id_list_[idx] == template_id){
            break;
        }
    }
    if(idx >= template_id_list_.size()){
        ROS_ERROR("Object ID: %d not found!", template_id);
    }else{
        ROS_INFO("Template %d attachment status: %d ",template_id,template_status_list_[idx]);

        if(template_status_list_[idx] == 1){ //Attached to robot
            /* First, define the DETACH object message*/
            moveit_msgs::AttachedCollisionObject detach_object;
            detach_object.object.id = boost::to_string((unsigned int)template_id);
            detach_object.object.operation = detach_object.object.REMOVE;

            ROS_INFO("Detaching the object %s from robot ",detach_object.object.id.c_str());
            aco_pub_.publish(detach_object);
            ros::spinOnce();
        }
        moveit_msgs::CollisionObject collision_object;
        collision_object.id              = boost::to_string((unsigned int)template_id);
        collision_object.header.frame_id = "/world";
        collision_object.operation       = collision_object.REMOVE;

        ROS_INFO("Removing the object %d from the environment", template_id);
        co_pub_.publish(collision_object);
    }
}
}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_template_nodelet, TemplateNodelet, ocs_template::TemplateNodelet, nodelet::Nodelet);
