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

    ROS_INFO(" Start reading database files");

    // which file are we reading
    if (!nhp.hasParam("r_grasps_filename"))
    {
        ROS_ERROR(" Did not find Right Grasp FILENAME parameter - using \"/vigir_template_library/grasp_templates/r_robotiq_grasp_library.csv\" as default");
    }
    if (!nhp.hasParam("l_grasps_filename"))
    {
        ROS_ERROR(" Did not find Left Grasp FILENAME parameter - using \"/vigir_template_library/grasp_templates/l_robotiq_grasp_library.csv\" as default");
    }
    if (!nhp.hasParam("ot_filename"))
    {
        ROS_ERROR(" Did not find Object Template FILENAME parameter - using \"/vigir_template_library/object_templates/object_templates.csv\" as default");
    }
    if (!nhp.hasParam("stand_filename"))
    {
        ROS_ERROR(" Did not find Ghost FILENAME parameter - using \"/vigir_template_library/robot_poses/atlas_v1_v3_joints_stand_poses.csv\" as default");
    }

    nhp.param<std::string>("r_grasps_filename", this->r_grasps_filename_,  ros::package::getPath("vigir_template_library")+"/grasp_templates/r_robotiq_grasp_library.csv");
    nhp.param<std::string>("l_grasps_filename", this->l_grasps_filename_,  ros::package::getPath("vigir_template_library")+"/grasp_templates/l_robotiq_grasp_library.csv");
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

    TemplateNodelet::loadGraspDatabase(this->r_grasps_filename_, "right");

    ROS_INFO("Right Grasp Database loaded");

    // Load the left hand specific grasping database
    nh.getParam("/l_hand_tf/gp_T_hand", gp_T_hand);
    gp_T_hand_.setOrigin(tf::Vector3(static_cast<double>(gp_T_hand[0]),static_cast<double>(gp_T_hand[1]),static_cast<double>(gp_T_hand[2])));
    gp_T_hand_.setRotation(tf::Quaternion(static_cast<double>(gp_T_hand[3]),static_cast<double>(gp_T_hand[4]),static_cast<double>(gp_T_hand[5]),static_cast<double>(gp_T_hand[6])));

    ROS_INFO("Right Graspit to Palm tf set");

    TemplateNodelet::loadGraspDatabase(this->l_grasps_filename_, "left");

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
    //template_type_list_.push_back(msg->template_type);	//Add the type of the template to be instantiated
    								//Could search for the path in the object_template_map_, but that sounds wasteful. Could we please put it in the message?
    template_list_.push_back(msg->template_path);
    pose_list_.push_back(msg->pose);
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
        //template_type_list_.erase(template_type_list_.begin()+index);	//Remove it
        template_list_.erase(template_list_.begin()+index);
        pose_list_.erase(pose_list_.begin()+index);
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

    cmd.template_id_list = template_id_list_;
    cmd.template_list = template_list_;
    //cmd.template_type_list = template_type_list_;	//I don't know if we want this information to come out of here; probably not
    cmd.pose = pose_list_;

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
        ROS_ERROR("NO GRASP DATABASE FILE FOUND: %s",file_name.c_str());
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

void TemplateNodelet::loadGraspDatabase(std::string& file_name, std::string hand_side){


    //LOADING HAND MODEL FOR JOINT NAMES (SHOULD WORK FOR ANY HAND)
    hand_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    hand_robot_model_ = hand_model_loader_->getModel();

    std::vector<std::string> joints;



    if(hand_robot_model_->hasJointModelGroup(hand_side+"_hand"))
    {
        joints.clear();
        joints = hand_robot_model_->getJointModelGroup(hand_side+"_hand")->getActiveJointModelNames();
    }else{
        ROS_WARN("NO JOINTS FOUND FOR %s HAND",hand_side.c_str());
    }

    ROS_INFO("%s %s hand model gotten, #actuated joints: %ld ",hand_side.c_str(), hand_robot_model_->getName().c_str(),joints.size() );

    for(int i = 0; i < joints.size(); i++)
        ROS_INFO("Joint %d: %s",i,joints[i].c_str());

    std::vector< std::vector <std::string> > db = readCSVFile(file_name);

    for(int i = 1; i < db.size(); i++) //STARTING FROM 1 SINCE FIRST LINE IS HEADER BEGINING WITH "#"
    {
        //ROS_INFO("Staring to read db line with %d elements",int(db[i].size()));

        unsigned int idx = 0;
        //ORDER IN FILE SHOULD MATCH ORDER IN READING
        /* template type,
         * grasp id,
         * approaching direction (vector (x,y,z), desired, minimal),
         * final grasp pose relative to template (x,y,z,qw,qx,qy,qz),
         * pre-grasp pose relative to template (x,y,z,qw,qx,qy,qz),
         * pre finger poses,
         * final finger poses
        */

        //TEMPLATE TYPE
        //ROS_INFO("Staring template type idx: %d",idx);
        unsigned int type = std::atoi(db[i][idx++].c_str());

        //GRASP ID
        //ROS_INFO("Staring grasp id idx: %d",idx);
        moveit_msgs::Grasp grasp;
        grasp.id                                           = db[i][idx++];

        //APPROACHING VECTOR
        //ROS_INFO("Staring approaching vector idx: %d",idx);
        grasp.pre_grasp_approach.direction.header.frame_id = hand_side[0]+"_hand"; //Should r_hand or l_hand also be a parameter?
        grasp.pre_grasp_approach.direction.vector.x        = std::atof(db[i][idx++].c_str());
        grasp.pre_grasp_approach.direction.vector.y        = std::atof(db[i][idx++].c_str());
        grasp.pre_grasp_approach.direction.vector.z        = std::atof(db[i][idx++].c_str());
        grasp.pre_grasp_approach.desired_distance          = std::atof(db[i][idx++].c_str());
        grasp.pre_grasp_approach.min_distance              = std::atof(db[i][idx++].c_str());

        //GRASP POSE
        //ROS_INFO("Staring grasp pose idx: %d",idx);
        grasp.grasp_pose.header.frame_id                   = hand_side[0]+"_hand";
        grasp.grasp_pose.pose.position.x                   = std::atof(db[i][idx++].c_str());
        grasp.grasp_pose.pose.position.y                   = std::atof(db[i][idx++].c_str());
        grasp.grasp_pose.pose.position.z                   = std::atof(db[i][idx++].c_str());
        grasp.grasp_pose.pose.orientation.w                = std::atof(db[i][idx++].c_str());
        grasp.grasp_pose.pose.orientation.x                = std::atof(db[i][idx++].c_str());
        grasp.grasp_pose.pose.orientation.y                = std::atof(db[i][idx++].c_str());
        grasp.grasp_pose.pose.orientation.z                = std::atof(db[i][idx++].c_str());

        //Static transformation to parent link /[r/l]_hand
        staticTransform(grasp.grasp_pose.pose);

        //PRE FINGER JOINT POSTURE
        //ROS_INFO("Staring pre finger joint posture idx: %d",idx);
        if(joints.size() > 0){
            grasp.pre_grasp_posture.joint_names.resize(joints.size());
            for(int j=0; j<joints.size();j++)
                grasp.pre_grasp_posture.joint_names[j] = joints.at(j);
            grasp.pre_grasp_posture.points.resize(1);
            grasp.pre_grasp_posture.points[0].positions.resize(joints.size());
            for(int j=0; j<joints.size();j++)
                grasp.pre_grasp_posture.points[0].positions[j] = std::atof(db[i][idx++].c_str());

            grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(3.0);

            //FINGER JOINT POSTURE
            //ROS_INFO("Staring final finger joint posture idx: %d",idx);
            grasp.grasp_posture.joint_names.resize(joints.size());
            for(int j=0; j<joints.size();j++)
                grasp.grasp_posture.joint_names[j] = joints.at(j);
            grasp.grasp_posture.points.resize(1);
            grasp.grasp_posture.points[0].positions.resize(joints.size());
            for(int j=0; j<joints.size();j++)
                grasp.grasp_posture.points[0].positions[j] = std::atof(db[i][idx++].c_str());

            grasp.grasp_posture.points[0].time_from_start = ros::Duration(3.0);
        }

        //RETREAT VECTOR (FIXING TO LIFT 10cm AFTER GRASPING)
        //ROS_INFO("Staring retreat vector idx: %d",idx);
        grasp.post_grasp_retreat.direction.header.frame_id = "world";
        grasp.post_grasp_retreat.direction.vector.z        = 1.0;
        grasp.post_grasp_retreat.min_distance              = 0.05;
        grasp.post_grasp_retreat.desired_distance          = 0.1;
        object_template_map_[type].grasps.insert(std::pair<unsigned int,moveit_msgs::Grasp>(std::atoi(grasp.id.c_str()),grasp));
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
	geometry_msgs::PoseStamped current_pose;
	std::map<unsigned int, VigirObjectTemplate>::iterator current_template;
	for (unsigned int i = 1; i < db.size(); ++i) {
		template_type = std::atoi(db[i][0].c_str());
		stand_pose_id = std::atoi(db[i][1].c_str());
		current_pose.header.frame_id = "/world";
		current_pose.header.stamp = ros::Time::now();

		current_pose.pose.position.x = std::atoi(db[i][2].c_str());
		current_pose.pose.position.y = std::atoi(db[i][3].c_str());
		current_pose.pose.position.z = std::atoi(db[i][4].c_str());

		current_pose.pose.orientation.w = std::atoi(db[i][5].c_str());
		current_pose.pose.orientation.x = std::atoi(db[i][6].c_str());
		current_pose.pose.orientation.y = std::atoi(db[i][7].c_str());
		current_pose.pose.orientation.z = std::atoi(db[i][8].c_str());

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
		
		current_template->second.stand_poses.insert(std::pair<unsigned int, geometry_msgs::PoseStamped> (stand_pose_id, current_pose));
		ROS_INFO_STREAM("Added stand information: type " << template_type << " pose id: " << stand_pose_id << " pose " << current_pose);
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
        object_template_map_.insert(std::pair<unsigned int,VigirObjectTemplate>(type,object_template));
        //ROS_INFO(" Inserting Object template type: %d with id: %d", object_template_map_[type].type, object_template_map_[type].id);
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
    vec_in.setOrigin(tf::Vector3(direction.vector.x,direction.vector.y,direction.vector.z));

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
			break;
		}
    }
	
	if (index == template_id_list_.size()){
        //ROS_ERROR_STREAM("Service requested template id " << req.template_id.data << " when no such id has been instantiated. Callback returning false.");
        ROS_ERROR("Service requested template id %d when no such id has been instantiated. Callback returning false.",req.template_id);
		return false;
    }

    res.template_state_information.template_id 	 = req.template_id;
    res.template_state_information.type_name 	 = object_template_map_[template_type].name;
    res.template_state_information.pose 		 = pose_list_[req.template_id];

    res.template_type_information.type_name      = object_template_map_[template_type].name;
    res.template_type_information.mass           = object_template_map_[template_type].mass;
    res.template_type_information.center_of_mass = object_template_map_[template_type].com;
	
	//Transfer all known grasps to response
	for (std::map<unsigned int,moveit_msgs::Grasp>::iterator it = object_template_map_[template_type].grasps.begin(); it != object_template_map_[template_type].grasps.end(); ++it) {
		res.template_type_information.grasps.push_back(it->second);
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
    res.template_type_information.geometry_marker.mesh_resource   = ros::package::getPath("vigir_template_library") + "/" + object_template_map_[template_type].path;
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
    for (std::map<unsigned int,moveit_msgs::Grasp>::iterator it2  = object_template_map_[req.template_id].grasps.begin();
                                                             it2 != object_template_map_[req.template_id].grasps.end();
                                                             ++it2, i++){
        res.grasp_information.grasps.push_back(it2->second);
    }
    i=0;
    for (std::map<unsigned int,geometry_msgs::PoseStamped>::iterator it2  = object_template_map_[req.template_id].stand_poses.begin();
                                                                     it2 != object_template_map_[req.template_id].stand_poses.end();
                                                                     ++it2, i++){
        res.grasp_information.stand_poses.push_back(it2->second);
    }
    return true;
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

    palm_pose.position.x = hand_vector.getX();
    palm_pose.position.y = hand_vector.getY();
    palm_pose.position.z = hand_vector.getZ();
    palm_pose.orientation.x = hand_quat.getX();
    palm_pose.orientation.y = hand_quat.getY();
    palm_pose.orientation.z = hand_quat.getZ();
    palm_pose.orientation.w = hand_quat.getW();
    return 0;
}

}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_template_nodelet, TemplateNodelet, ocs_template::TemplateNodelet, nodelet::Nodelet);
