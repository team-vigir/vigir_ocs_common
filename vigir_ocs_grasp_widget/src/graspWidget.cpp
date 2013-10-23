#include "graspWidget.h"
#include "ui_graspWidget.h"
#include <ros/package.h>
#include <QColor>
#include <QProgressBar>
#include <QSlider>

//grasp_testing grasp_testing_simple.launch

graspWidget::graspWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::graspWidget)
    , selected_template_id_(-1)
    , selected_grasp_id_(-1)
    , show_grasp_(false)
{
    // setup UI
    ui->setupUi(this);
    ui->templateBox->setDisabled(true);
    ui->graspBox->setDisabled(true);
    ui->performButton->setDisabled(true);
    //ui->templateButton->setDisabled(true);
    //ui->releaseButton->setDisabled(true);

    // initialize arguments from parameter server
    ros::NodeHandle nhp("~");
    nhp.param<std::string>("grasp_widget/hand",hand,"left"); // private parameter
    nh_.param<std::string>("/flor/ocs/grasp/hand_type",hand_type,"sandia"); // global parameter
    ROS_ERROR("  Grasp widget using %s hand (%s)",hand.c_str(), hand_type.c_str());

    // initialize path variables for template/grasp databases
    std::string templatePath = (ros::package::getPath("templates"))+"/";
    std::cout << "--------------<" << templatePath << ">\n" << std::endl;
    template_dir_path_ = QString(templatePath.c_str());
    template_id_db_path_ = template_dir_path_+QString("grasp_templates.txt");
    if(hand_type == "irobot")
        grasp_db_path_ = template_dir_path_+QString("grasp_library_irobot.grasp");
    else
        grasp_db_path_ = template_dir_path_+QString("grasp_library.grasp");

    // initialize variables
    currentGraspMode = 0;
    templateMatchDone = false;

    // read from databases
    initTemplateIdMap();
    initGraspDB();

    // initialize template subscribers and publishers
    template_list_sub_           = nh_.subscribe<flor_ocs_msgs::OCSTemplateList>(    "/template/list",                    5, &graspWidget::processTemplateList, this );
    template_match_feedback_sub_ = nh_.subscribe<flor_grasp_msgs::TemplateSelection>("/template/template_match_feedback", 1, &graspWidget::templateMatchFeedback, this );
    grasp_state_sub_             = nh_.subscribe<flor_grasp_msgs::GraspState>(       "/template/active_state",            1, &graspWidget::graspStateReceived,  this );

    grasp_selection_pub_        = nh_.advertise<flor_grasp_msgs::GraspSelection>(    "/template/grasp_selection",        1, false);
    grasp_release_pub_          = nh_.advertise<flor_grasp_msgs::GraspSelection>(    "/template/release_grasp" ,         1, false);
    grasp_mode_command_pub_     = nh_.advertise<flor_grasp_msgs::GraspState>(         "/template/grasp_mode_command",     1, false);
    template_match_request_pub_ = nh_.advertise<flor_grasp_msgs::TemplateSelection>( "/template/template_match_request", 1, false );

    // create subscribers for grasp status
    std::stringstream finger_joint_name;
    if(hand == "left")
    {
        this->setWindowTitle(QString::fromStdString("Left Hand Grasp Widget"));
        robot_status_sub_           = nh_.subscribe<flor_ocs_msgs::OCSRobotStatus>( "/grasp_control/l_hand/grasp_status",1, &graspWidget::robotStatusCB,  this );
        ghost_hand_pub_             = nh_.advertise<geometry_msgs::PoseStamped>(     "/ghost_left_hand_pose",             1, false);
        ghost_hand_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(        "/ghost_left_hand/joint_states",     1, false); // /ghost_left_hand/joint_states

        // We first subscribe to the JointState messages
        joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>( "/grasp_control/l_hand/sandia_states", 2, &graspWidget::jointStatesCB, this );
    }
    else
    {
        this->setWindowTitle(QString::fromStdString("Right Hand Grasp Widget"));
        robot_status_sub_           = nh_.subscribe<flor_ocs_msgs::OCSRobotStatus>( "/grasp_control/r_hand/grasp_status",1, &graspWidget::robotStatusCB,  this );
        ghost_hand_pub_             = nh_.advertise<geometry_msgs::PoseStamped>(     "/ghost_right_hand_pose",            1, false);
        ghost_hand_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(        "/ghost_right_hand/joint_states",    1, false); // /ghost_right_hand/joint_states

        // We first subscribe to the JointState messages
        joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>( "/grasp_control/r_hand/sandia_states", 2, &graspWidget::jointStatesCB, this );
    }

    // publisher to color the hand links
    hand_link_color_pub_        = nh_.advertise<flor_ocs_msgs::OCSLinkColor>("/link_color", 1, false);

    // find robot status message code csv file
    std::string code_path_ = (ros::package::getPath("flor_ocs_msgs"))+"/include/flor_ocs_msgs/messages.csv";
    std::cout << code_path_ << std::endl;
    robot_status_codes_.loadErrorMessages(code_path_);

    timer.start(33, this);
}
//SetStylesheet to change on the fly

graspWidget::~graspWidget()
{
    delete ui;
}

void graspWidget::timerEvent(QTimerEvent *event)
{
    // make sure that we don't show the grasp hands if the user doesn't want to see them
    if(!ui->graspBox->isEnabled() || !show_grasp_)
        hideHand();

    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

void graspWidget::templateMatchFeedback (const flor_grasp_msgs::TemplateSelection::ConstPtr& feedback)
{
    // provide feedback about template grasp confidence by changing the color of the move to template button
    QPalette pal = ui->templateButton->palette();
    if(feedback->confidence.data >= 85)
        pal.setColor(QPalette::Button,Qt::green);
    else if(feedback->confidence.data >=60)
        pal.setColor(QPalette::Button,Qt::yellow);
    else
        pal.setColor(QPalette::Button,Qt::red);
    ui->templateButton->setPalette(pal);
    ui->templateButton->setAutoFillBackground(true);
    std::cout << "Template confidence received and found to be " << (int)feedback->confidence.data << std::endl;
    templateMatchDone = true; // is this still being used?
}

void graspWidget::graspStateReceived (const flor_grasp_msgs::GraspState::ConstPtr& graspState)
{
    //std::cout << "Grasp State message received" << graspState << std::endl;
    uint8_t mode  = (graspState->grasp_state.data&0xF0) >> 4;
    uint8_t state = graspState->grasp_state.data&0x0F;
    setProgressLevel(graspState->grip.data);
    ui->thumbGraf->setValue(graspState->thumb_effort.data);

    ui->userSlider->setValue(graspState->grip.data);
    ui->userSlider_2->setValue(graspState->thumb_effort.data);
    //std::cout << "     mode=" << uint32_t(mode) << "   state="<< uint32_t(state) << std::endl;
    switch(mode)
    {
    case flor_grasp_msgs::GraspState::GRASP_MODE_NONE:
        ui->currentStateLabel->setText("idle");
        currentGraspMode = 0;
        break;
    case flor_grasp_msgs::GraspState::TEMPLATE_GRASP_MODE:
        if(currentGraspMode != 1)
        {
            ui->manualRadio->setChecked(false);
            ui->templateRadio->setChecked(true);
            initTemplateMode();
            currentGraspMode = 1;
        }
        switch(state)
        {
        case flor_grasp_msgs::GraspState::GRASP_STATE_NONE:
            ui->currentStateLabel->setText("State Unknown");
            break;
        case flor_grasp_msgs::GraspState::GRASP_INIT:
            ui->currentStateLabel->setText("init");
            break;
        case flor_grasp_msgs::GraspState::APPROACHING:
            ui->currentStateLabel->setText("approaching");
            break;
        case flor_grasp_msgs::GraspState::SURROUNDING:
            ui->currentStateLabel->setText("surrounding");
            break;
        case flor_grasp_msgs::GraspState::GRASPING:
            ui->currentStateLabel->setText("grasping");
            break;
        case flor_grasp_msgs::GraspState::MONITORING:
            ui->currentStateLabel->setText("monitoring");
            break;
        case flor_grasp_msgs::GraspState::OPENING:
            ui->currentStateLabel->setText("opening");
            break;
        case flor_grasp_msgs::GraspState::GRASP_ERROR:
        default:
            ui->currentStateLabel->setText("template error");
            break;
        }
        break;
    case flor_grasp_msgs::GraspState::MANUAL_GRASP_MODE:
        if(currentGraspMode != 2)
        {
            ui->manualRadio->setChecked(true);
            ui->templateRadio->setChecked(false);
            currentGraspMode = 2;
        }
        ui->currentStateLabel->setText("manual");
        break;
    default:
        ui->currentStateLabel->setText("MODE ERROR");
        break;
    }
}


void graspWidget::processTemplateList( const flor_ocs_msgs::OCSTemplateList::ConstPtr& list)
{
    std::cout << "Template list received containing " << list->template_id_list.size() << " elements" << std::cout;
    // save last template list
    last_template_list_ = *list;

    // enable boxes and buttons
    if(list->template_list.size() > 0)
    {
        ui->templateBox->setDisabled(false);
        ui->graspBox->setDisabled(false);
        ui->performButton->setDisabled(false);
    }

    bool was_empty = ui->templateBox->count() == 0 ? true : false;

    QString currentItem = ui->templateBox->currentText();
    //ui->templateBox->clear();

    // populate template combobox
    for(int i = 0; i < list->template_list.size(); i++)
    {
        // remove the .mesh string from the template name
        std::string templateName = list->template_list[i];
        if(templateName.size() > 5 && templateName.substr(templateName.size()-5,5) == ".mesh")
            templateName = templateName.substr(0,templateName.size()-5);
        // add the template
        templateName = boost::to_string((int)list->template_id_list[i])+std::string(": ")+templateName;

        std::cout << "template item " << (int)list->template_id_list[i] << " has name " << templateName << std::endl;

        // add the template to the box if it doesn't exist
        if(ui->templateBox->count() < i+1)
        {
            ui->templateBox->addItem(QString::fromStdString(templateName));
        } // update existing templates
        else if( ui->templateBox->itemText(i).toStdString() != templateName)
        {
            ui->templateBox->setItemText(i,QString::fromStdString(templateName));
        }
    }

    for(int i = list->template_list.size(); i < ui->templateBox->count(); i++)
        ui->templateBox->removeItem(i);

    if(selected_template_id_ != -1 && ui->templateBox->findText(currentItem) == -1)
    {
        hideHand();
        ui->graspBox->clear();
        selected_template_id_ = -1;
        selected_grasp_id_ = -1;
        ui->graspBox->setEnabled(false);
    }
    else
    {
        if(was_empty && ui->templateBox->count() > 0)
        {
            //ROS_ERROR("Seleting template 0");
            ui->templateBox->setCurrentIndex(0);
            on_templateBox_activated(ui->templateBox->itemText(0));
            on_templateRadio_clicked();
            selected_template_id_ = 0;
        }

        if(selected_grasp_id_ != -1 && show_grasp_)
            publishHandPose(selected_grasp_id_);
    }
}

void graspWidget::initTemplateMode()
{
    if(last_template_list_.template_id_list.size() > 0)
    {
        ui->templateBox->setDisabled(false);
        ui->graspBox->setDisabled(false);
    }
}

//currentStateLabel
void graspWidget::setProgressLevel(uint8_t level)
{
    //std::cout << "setting fill level to be " << (int)level << std::endl;
    if(level >=100)
    {
        ui->closedGraph->setValue(100);
        ui->forceGraph->setValue((int)(level-100));
    }
    else
    {
        ui->closedGraph->setValue(level);
        ui->forceGraph->setValue(0);
    }
}

void graspWidget::initTemplateIdMap()
{
    std::vector< std::vector<QString> > db = readTextDBFile(template_id_db_path_);

    for(int i = 0; i < db.size(); i++)
    {
        bool ok;
        unsigned char id = db[i][0].toUInt(&ok, 10) & 0x000000ff;
        std::string templatePath(db[i][1].toUtf8().constData());
        std::cout << "-> Adding template (" << templatePath << ") to id (" << (unsigned int)id << ") map" << std::endl;
        template_id_map_.insert(std::pair<unsigned char,std::string>(id,templatePath));
    }
}

// will return a vector with rows, each row containing a QStringList with all columns
std::vector< std::vector<QString> > graspWidget::readTextDBFile(QString path)
{
    std::vector< std::vector<QString> > ret;
    QFile file(path);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QTextStream in(&file);
        while (!in.atEnd())
        {
            QString line = in.readLine();
            if(line[0] != '#')
            {
                std::vector<QString> row;
                QStringList strings;
                strings = line.split(",");
                // remove whitespaces
                for(int i = 0; i < strings.size(); i++)
                {
                    QString str = strings.at(i);
                    str.replace(" ","");
                    row.push_back(str);
                }
                ret.push_back(row);
            }
        }
    }
    return ret;
}

void graspWidget::initGraspDB()
{
    if(hand_type == "irobot")
    {
        std::vector< std::vector<QString> > db = readTextDBFile(grasp_db_path_);
        for(int i = 0; i < db.size(); i++)
        {
            bool ok;
            // [0] grasp id, [1] template type, [2] hand, [3] initial grasp type, [4] DISCARD, [5-9] finger joints (5), [10] DISCARD, [11-17] final grasp pose relative to template (x,y,z,qx,qy,qz,qw), [18] DISCARD, [19-25] pre-grasp pose relative to template (x,y,z,qx,qy,qz,qw)
            GraspDBItem grasp;
            //std::cout << "-> Adding grasp to grasp DB" << std::endl;
            grasp.grasp_id = db[i][0].toUInt(&ok, 10) & 0x0000ffff;
            std::cout << "id: " << (unsigned int)grasp.grasp_id << std::endl;

            grasp.template_type = db[i][1].toUInt(&ok, 10) & 0x000000ff;
            std::cout << "template type: " << (unsigned int)grasp.template_type << std::endl;

            grasp.template_name = template_id_map_.find(grasp.template_type)->second;
            std::cout << "template name: " << grasp.template_name << std::endl;

            grasp.hand = db[i][2].toUtf8().constData();
            std::cout << "hand: " << grasp.hand << std::endl;

            grasp.initial_grasp_type = db[i][3].toUtf8().constData();
            std::cout << "initial grasp type: " << grasp.initial_grasp_type << std::endl;

            //std::cout << "finger joints: ";
            for(int j = 0; j < 5; j++)
            {
                // need to ignore fourth joint in the grasp file [3], and send the same value as [4] in its place
                //                joint_states.name.push_back(hand+"_finger[0]_proximal");
                //                joint_states.name.push_back(hand+"_finger[1]_proximal");
                //                joint_states.name.push_back(hand+"_finger[2]_proximal");
                //                joint_states.name.push_back(hand+"_finger[0]_base_rotation"); // .grasp finger position [4] -> IGNORE [3], use [4] for both
                //                joint_states.name.push_back(hand+"_finger[1]_base_rotation");// .grasp finger position [4]
                if(j == 3)
                    grasp.finger_joints[j] = db[i][j+5+1].toFloat(&ok);
                else
                    grasp.finger_joints[j] = db[i][j+5].toFloat(&ok);
            }
            // need to set distal joints as 0
            //                joint_states.name.push_back(hand+"_finger[0]_distal"); // 0 for now
            //                joint_states.name.push_back(hand+"_finger[1]_distal");// 0 for now
            //                joint_states.name.push_back(hand+"_finger[2]_distal");// 0 for now
            grasp.finger_joints[5] = 0;
            grasp.finger_joints[6] = 0;
            grasp.finger_joints[7] = 0;
            //std::cout << std::endl;

            grasp.final_pose.position.x = db[i][11].toFloat(&ok);
            grasp.final_pose.position.y = db[i][12].toFloat(&ok);
            grasp.final_pose.position.z = db[i][13].toFloat(&ok);
            grasp.final_pose.orientation.w = db[i][14].toFloat(&ok);
            grasp.final_pose.orientation.x = db[i][15].toFloat(&ok);
            grasp.final_pose.orientation.y = db[i][16].toFloat(&ok);
            grasp.final_pose.orientation.z = db[i][17].toFloat(&ok);
            //std::cout << "final pose: " << grasp.final_pose.position.x << ", " << grasp.final_pose.position.y << ", " << grasp.final_pose.position.z << ", " <<
            //             grasp.final_pose.orientation.x << ", " << grasp.final_pose.orientation.y << ", " << grasp.final_pose.orientation.y << ", " << grasp.final_pose.orientation.w << std::endl;

            grasp.pre_grasp_pose.position.x = db[i][19].toFloat(&ok);
            grasp.pre_grasp_pose.position.y = db[i][20].toFloat(&ok);
            grasp.pre_grasp_pose.position.z = db[i][21].toFloat(&ok);
            grasp.pre_grasp_pose.orientation.w = db[i][22].toFloat(&ok);
            grasp.pre_grasp_pose.orientation.x = db[i][23].toFloat(&ok);
            grasp.pre_grasp_pose.orientation.y = db[i][24].toFloat(&ok);
            grasp.pre_grasp_pose.orientation.z = db[i][25].toFloat(&ok);
            //std::cout << "final pose: " << grasp.pre_grasp_pose.position.x << ", " << grasp.pre_grasp_pose.position.y << ", " << grasp.pre_grasp_pose.position.z << ", " <<
            //             grasp.pre_grasp_pose.orientation.x << ", " << grasp.pre_grasp_pose.orientation.y << ", " << grasp.pre_grasp_pose.orientation.y << ", " << grasp.pre_grasp_pose.orientation.w << std::endl;

            grasp_db_.push_back(grasp);
        }
    }
    else // sandia
    {
        std::vector< std::vector<QString> > db = readTextDBFile(grasp_db_path_);
        for(int i = 0; i < db.size(); i++)
        {
            bool ok;
            // [0] grasp id, [1] template type, [2] hand, [3] initial grasp type, [4] DISCARD, [5-16] finger joints (12), [17] DISCARD, [18-24] final grasp pose relative to template (x,y,z,qx,qy,qz,qw), [25] DISCARD, [26-32] pre-grasp pose relative to template (x,y,z,qx,qy,qz,qw)
            GraspDBItem grasp;
            //std::cout << "-> Adding grasp to grasp DB" << std::endl;
            grasp.grasp_id = db[i][0].toUInt(&ok, 10) & 0x0000ffff;
            std::cout << "id: " << (unsigned int)grasp.grasp_id << std::endl;

            grasp.template_type = db[i][1].toUInt(&ok, 10) & 0x000000ff;
            std::cout << "template type: " << (unsigned int)grasp.template_type << std::endl;

            grasp.template_name = template_id_map_.find(grasp.template_type)->second;
            std::cout << "template name: " << grasp.template_name << std::endl;

            grasp.hand = db[i][2].toUtf8().constData();
            std::cout << "hand: " << grasp.hand << std::endl;

            grasp.initial_grasp_type = db[i][3].toUtf8().constData();
            std::cout << "initial grasp type: " << grasp.initial_grasp_type << std::endl;

            //std::cout << "finger joints: ";
            for(int j = 0; j < 12; j++)
            {

                // Graspit outputs the fingers in different order, and these were copied into .grasp library
                // We need to swap f0 and f2, which this code does
                if(j < 3)
                    grasp.finger_joints[j+6] = db[i][j+5].toFloat(&ok);//grasp_spec.finger_poses.f2[i]= db[i][j].toFloat(&ok); //joints from the pinky
                else if (j > 5 && j < 9)
                    grasp.finger_joints[j-6] = db[i][j+5].toFloat(&ok);//grasp_spec.finger_poses.f0[i-6]= db[i][j].toFloat(&ok); //joints from the index
                else// if ((j-5) > 2 && (j-5) < 6)
                    grasp.finger_joints[j] = db[i][j+5].toFloat(&ok);//grasp_spec.finger_poses.f1[i-3]= db[i][j].toFloat(&ok); //joints from middle and thumb
                //else //9,10,11
                //    grasp.finger_joints[(j-5)] = db[i][j].toFloat(&ok);//grasp_spec.finger_poses.f3[i-9]= db[i][j].toFloat(&ok); //joints from thumb
                //grasp.finger_joints[j-5] = db[i][j].toFloat(&ok); // old code, using Graspit order
                //std::cout << grasp.finger_joints[j-5] << ",";
            }
            //std::cout << std::endl;

            grasp.final_pose.position.x = db[i][18].toFloat(&ok);
            grasp.final_pose.position.y = db[i][19].toFloat(&ok);
            grasp.final_pose.position.z = db[i][20].toFloat(&ok);
            grasp.final_pose.orientation.w = db[i][21].toFloat(&ok);
            grasp.final_pose.orientation.x = db[i][22].toFloat(&ok);
            grasp.final_pose.orientation.y = db[i][23].toFloat(&ok);
            grasp.final_pose.orientation.z = db[i][24].toFloat(&ok);
            //std::cout << "final pose: " << grasp.final_pose.position.x << ", " << grasp.final_pose.position.y << ", " << grasp.final_pose.position.z << ", " <<
            //             grasp.final_pose.orientation.x << ", " << grasp.final_pose.orientation.y << ", " << grasp.final_pose.orientation.y << ", " << grasp.final_pose.orientation.w << std::endl;

            grasp.pre_grasp_pose.position.x = db[i][26].toFloat(&ok);
            grasp.pre_grasp_pose.position.y = db[i][27].toFloat(&ok);
            grasp.pre_grasp_pose.position.z = db[i][28].toFloat(&ok);
            grasp.pre_grasp_pose.orientation.w = db[i][29].toFloat(&ok);
            grasp.pre_grasp_pose.orientation.x = db[i][30].toFloat(&ok);
            grasp.pre_grasp_pose.orientation.y = db[i][31].toFloat(&ok);
            grasp.pre_grasp_pose.orientation.z = db[i][32].toFloat(&ok);
            //std::cout << "final pose: " << grasp.pre_grasp_pose.position.x << ", " << grasp.pre_grasp_pose.position.y << ", " << grasp.pre_grasp_pose.position.z << ", " <<
            //             grasp.pre_grasp_pose.orientation.x << ", " << grasp.pre_grasp_pose.orientation.y << ", " << grasp.pre_grasp_pose.orientation.y << ", " << grasp.pre_grasp_pose.orientation.w << std::endl;

            grasp_db_.push_back(grasp);
        }
    }
}


void graspWidget::sendManualMsg(uint8_t level, uint8_t thumb)
{
    flor_grasp_msgs::GraspState cmd;
    cmd.grip.data         = level;
    cmd.thumb_effort.data = thumb;
    cmd.grasp_state.data = 4; // leave as current command
    if (ui->graspBox->currentText() == QString("CYLINDRICAL"))  cmd.grasp_state.data = 0;
    if (ui->graspBox->currentText() == QString("PRISMATIC"))    cmd.grasp_state.data = 1;
    if (ui->graspBox->currentText() == QString("SPHERICAL"))    cmd.grasp_state.data = 2;
    cmd.grasp_state.data += (flor_grasp_msgs::GraspState::MANUAL_GRASP_MODE)<<4;
    grasp_mode_command_pub_.publish(cmd);
    std::cout << "Sent Manual mode message ("<< uint32_t(cmd.grasp_state.data) << ") with " <<  uint32_t(cmd.grip.data) << " manual grip level and " << uint32_t(cmd.thumb_effort.data) <<  " thumb effort to " << hand << " hand" << std::endl;
}

void graspWidget::on_userSlider_sliderReleased()
{
    if(ui->manualRadio->isChecked())
    {
        setProgressLevel(ui->userSlider->value());
        sendManualMsg(ui->userSlider->value(), ui->userSlider_2->value());
    }
    else if (ui->templateRadio->isChecked())
    {
        if(ui->userSlider->value() > 100)
        {
            flor_grasp_msgs::GraspState msg;
            msg.grasp_state.data = ((flor_grasp_msgs::GraspState::TEMPLATE_GRASP_MODE)<<4) + 4;
            msg.grip.data        = ui->userSlider->value();
            msg.thumb_effort.data= ui->userSlider_2->value();
            grasp_mode_command_pub_.publish(msg);
            std::cout << "Adjust feedforward to " << int32_t(ui->userSlider->value()) << " with state=" << uint32_t(msg.grasp_state.data) << std::endl;
        }
        else
        {
            std::cout << "Only relevant in template mode if the feedforward is set!  New position is " << ui->userSlider->value() << std::endl;
            flor_grasp_msgs::GraspState msg;
            msg.grasp_state.data = ((flor_grasp_msgs::GraspState::TEMPLATE_GRASP_MODE)<<4) + 4;
            msg.grip.data        = 100; // can't undo grasp closure in template mode, but need to send message to clear feedforward
            msg.thumb_effort.data= ui->userSlider_2->value();
            grasp_mode_command_pub_.publish(msg);
        }
    }
    else
    {
        std::cout << "slider changed while not in any control mode. New position is " << ui->userSlider->value() << std::endl;
    }
}

void graspWidget::on_userSlider_2_sliderReleased()
{
    if(ui->manualRadio->isChecked())
    {
        ui->thumbGraf->setValue(ui->userSlider_2->value());
        sendManualMsg(ui->userSlider->value(), ui->userSlider_2->value());
    }
    else if (ui->templateRadio->isChecked())
    {
        if(ui->userSlider->value() > 100)
        {
            flor_grasp_msgs::GraspState msg;
            msg.grasp_state.data  = ((flor_grasp_msgs::GraspState::TEMPLATE_GRASP_MODE)<<4) + 4;
            msg.grip.data         = ui->userSlider->value();
            msg.thumb_effort.data = ui->userSlider_2->value();
            grasp_mode_command_pub_.publish(msg);
            std::cout << "Adjust thumb feedforward to " << int32_t(ui->userSlider_2->value()) << " with state=" << uint32_t(msg.grasp_state.data) << std::endl;
        }
        else
        {
            std::cout << "Only relevant in template mode if the thumb feedforward is set!  New position is " << ui->userSlider_2->value() << std::endl;
            flor_grasp_msgs::GraspState msg;
            msg.grasp_state.data  = ((flor_grasp_msgs::GraspState::TEMPLATE_GRASP_MODE)<<4) + 4;
            msg.grip.data         = 100; // can't undo grasp closure in template mode, but need to send message to clear feedforward
            msg.thumb_effort.data = ui->userSlider_2->value();
            grasp_mode_command_pub_.publish(msg);
        }
    }
    else
    {
        std::cout << "slider changed while not in any control mode. New position is " << ui->userSlider_2->value() << std::endl;
    }
}

void graspWidget::on_releaseButton_clicked()
{
    std::cout << "Release the grasp requested" << std::endl;
    ui->userSlider->setValue(0);
    ui->userSlider_2->setValue(0);
    //flor_grasp_msgs::GraspState msg;
    //msg.grasp_state.data = 0;
    //msg.grip.data = 0;
    //grasp_mode_command_pub_.publish(msg);

    flor_grasp_msgs::GraspSelection grasp_msg;
    grasp_msg.header.stamp=ros::Time::now();
    grasp_msg.grasp_id.data      = 0;
    grasp_msg.template_id.data   = 0;
    grasp_msg.template_type.data = 0;
    grasp_release_pub_.publish(grasp_msg);
    //ui->templateButton->setDisabled(true); // unable to move
    //ui->releaseButton->setDisabled(true);
}

void graspWidget::on_templateButton_clicked()
{
    hideHand();
    std::cout << "template match requested..." << std::endl;
    flor_grasp_msgs::TemplateSelection msg;
    int graspID = ui->graspBox->currentText().toInt();
    for(int index = 0; index < grasp_db_.size(); index++)
    {
        if(grasp_db_[index].grasp_id == graspID)
            msg.template_type.data = grasp_db_[index].template_type;
    }
    msg.template_id.data = ui->templateBox->currentIndex();
    msg.pose.pose = last_template_list_.pose[ui->templateBox->currentIndex()].pose;
    template_match_request_pub_.publish(msg);
}

void graspWidget::on_performButton_clicked()
{
    on_templateButton_clicked();
    std::cout << "Performing grasp" << std::endl;
    flor_grasp_msgs::GraspSelection msg;
    msg.header.frame_id = "/world";
    int graspID = ui->graspBox->currentText().toInt();
    msg.grasp_id.data = graspID;
    msg.template_id.data = ui->templateBox->currentIndex();
    for(int index = 0; index < grasp_db_.size(); index++)
    {
        if(grasp_db_[index].grasp_id == graspID)
            msg.template_type.data = grasp_db_[index].template_type;
    }
    grasp_selection_pub_.publish(msg);
    //ui->templateButton->setEnabled(true); // able to move
    //ui->releaseButton->setEnabled(true); // able to release
}

void graspWidget::on_templateBox_activated(const QString &arg1)
{
    // update the selected template id
    QString template_id = ui->templateBox->currentText();
    template_id.remove(template_id.indexOf(": "),template_id.length()-template_id.indexOf(": "));
    selected_template_id_ = template_id.toInt();

    std::cout << "updating the grasp widget grasp selection box contents" << std::endl;
    // clean grasp box
    ui->graspBox->clear();
    selected_grasp_id_ = -1;

    // add grasps to the grasp combo box
    for(int index = 0; index < grasp_db_.size(); index++)
    {
        QString tmp = arg1;
        tmp.remove(0,tmp.indexOf(": ")+2);
        std::cout << "comparing db " << grasp_db_[index].template_name << " to " << tmp.toStdString() << std::endl;

        if(grasp_db_[index].template_name == tmp.toStdString() && grasp_db_[index].hand == hand)
        {
            std::cout << "Found grasp for template" << std::endl;
            ui->graspBox->addItem(QString::number(grasp_db_[index].grasp_id));
        }
    }

    if(ui->templateBox->count() > 0)
        selected_grasp_id_ = ui->graspBox->itemText(0).toInt();

    if (ui->manualRadio->isChecked())
    {
        ui->graspBox->addItem(QString("CYLINDRICAL"));
        ui->graspBox->addItem(QString("PRISMATIC"));
        ui->graspBox->addItem(QString("SPHERICAL"));
    }
}

void graspWidget::on_graspBox_activated(const QString &arg1)
{
    std::cout << " grasp selection = " << arg1.toStdString() << std::endl;
    if (ui->manualRadio->isChecked())
    {
        flor_grasp_msgs::GraspState msg;
        msg.grasp_state.data = 4; // leave as current command
        if (arg1 == QString("CYLINDRICAL"))  msg.grasp_state.data = 0;
        if (arg1 == QString("PRISMATIC"))    msg.grasp_state.data = 1;
        if (arg1 == QString("SPHERICAL"))    msg.grasp_state.data = 2;
        msg.grasp_state.data += (flor_grasp_msgs::GraspState::MANUAL_GRASP_MODE)<<4;
        msg.grip.data         = ui->userSlider->value();
        msg.thumb_effort.data = ui->userSlider_2->value();
        grasp_mode_command_pub_.publish(msg);
        std::cout << "Sent Manual mode message ("<< uint32_t(msg.grasp_state.data) << ") with " <<  uint32_t(msg.grip.data) << " manual grip level and " << uint32_t(msg.thumb_effort.data) <<  " thumb effort to " << hand << " hand" << std::endl;
    }
    else
    {
        selected_grasp_id_ = arg1.toInt();
        publishHandPose(arg1.toUInt());
    }
}

void graspWidget::on_templateRadio_clicked()
{
    for (uint32_t ndx=0; ndx < ui->graspBox->count(); ++ndx)
    {
        if ( (ui->graspBox->itemText(ndx) == QString("CYLINDRICAL")) ||
             (ui->graspBox->itemText(ndx) == QString("PRISMATIC")) ||
             (ui->graspBox->itemText(ndx) == QString("SPHERICAL")) ||
             (ui->graspBox->itemText(ndx) == QString("CURRENT")))
        {

            std::string str = ui->graspBox->itemText(ndx).toStdString();
            ROS_INFO(" Removing item at %d (%s)", ndx, str.c_str());
            ui->graspBox->removeItem(ndx);
            --ndx;
        }
    }
    selected_template_id_ = -1;
    if (ui->graspBox->count() < 1)
    {
        ui->graspBox->setDisabled(true); // nothing to select
    }
    flor_grasp_msgs::GraspState msg;
    msg.grasp_state.data  = (flor_grasp_msgs::GraspState::TEMPLATE_GRASP_MODE)<<4;
    msg.grip.data         = 0;
    msg.thumb_effort.data = 0;
    grasp_mode_command_pub_.publish(msg);
    std::cout << "Sent Template mode message ("<< uint32_t(msg.grasp_state.data) << ") with " <<  uint32_t(msg.grip.data) << " manual grip level and " << uint32_t(msg.thumb_effort.data) <<  " thumb effort to " << hand << " hand" << std::endl;

}

void graspWidget::on_manualRadio_clicked()
{
    bool addCylindrical= true;
    bool addPrismatic  = true;
    bool addSpherical  = true;
    bool addCurrent    = true;

    for (uint32_t ndx=0; ndx < ui->graspBox->count(); ++ndx)
    {
        if (ui->graspBox->itemText(ndx) == QString("CYLINDRICAL"))  addCylindrical= false;
        if (ui->graspBox->itemText(ndx) == QString("PRISMATIC"))    addPrismatic  = false;
        if (ui->graspBox->itemText(ndx) == QString("SPHERICAL"))    addSpherical  = false;
        if (ui->graspBox->itemText(ndx) == QString("CURRENT"))      addCurrent    = false;
    }
    if (addCylindrical) ui->graspBox->addItem(QString("CYLINDRICAL"));
    if (addPrismatic)   ui->graspBox->addItem(QString("PRISMATIC"));
    if (addSpherical)   ui->graspBox->addItem(QString("SPHERICAL"));
    if (addCurrent)     ui->graspBox->addItem(QString("CURRENT"));
    ui->graspBox->setDisabled(false);


    flor_grasp_msgs::GraspState msg;
    msg.grasp_state.data  = (flor_grasp_msgs::GraspState::MANUAL_GRASP_MODE)<<4;
    msg.grasp_state.data += 4; // no grasp type chosen (force selection) (default to keeping old terminal values)
    msg.grip.data         = ui->userSlider->value();
    msg.thumb_effort.data = ui->userSlider_2->value();
    grasp_mode_command_pub_.publish(msg);
    std::cout << "Sent Manual mode message ("<< uint32_t(msg.grasp_state.data) << ") with " <<  uint32_t(msg.grip.data) << " manual grip level and " << uint32_t(msg.thumb_effort.data) <<  " thumb effort to " << hand << " hand" << std::endl;
}

void graspWidget::robotStatusCB(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg)
{
    uint16_t code;
    uint8_t  severity;
    RobotStatusCodes::codes(msg->status,code,severity);
    //ROS_INFO("  grasp widget code=%d, severity=%d",code,severity);
    ui->robot_status_->setText(robot_status_codes_.str(code).c_str());
}

void graspWidget::jointStatesCB( const sensor_msgs::JointState::ConstPtr& joint_states )
{
    if(hand_type == "irobot")
    {

    }
    else
    {
        //Index 	Name            Link
        //0         right_f0_j0 	Palm index base
        //1         right_f0_j1 	Index proximal
        //2         right_f0_j2 	Index distal
        //3         right_f1_j0 	Palm middle base
        //4         right_f1_j1 	Middle proximal
        //5         right_f1_j2 	Middle distal
        //6         right_f2_j0 	Palm little base
        //7         right_f2_j1 	Little proximal
        //8         right_f2_j2 	Little distal
        //9         right_f3_j0 	Palm cylinder
        //10        right_f3_j1 	Thumb proximal
        //11        right_f3_j2 	Thumb distal

        double min_feedback = 0, max_feedback = 1.0;
        for(int i = 0; i < joint_states->name.size(); i++)
        {
            // get the joint name to figure out the color of the links
            std::string joint_name = joint_states->name[i].c_str();

            // velocity represents tactile feedback
            double feedback = joint_states->velocity[i];

            // NOTE: this is SPECIFIC to atlas hands, IT IS NOT GENERAL
            std::string link_name;
            size_t found = joint_name.find('j');
            if( found != std::string::npos )
                link_name = joint_name.erase(found,1);

            //ROS_ERROR("Applying color to %s",link_name.c_str());
            // calculate color intensity based on min/max feedback
            unsigned char color_intensity = (unsigned char)((feedback - min_feedback)/(max_feedback-min_feedback) * 255.0);
            publishLinkColor(link_name,color_intensity,255-color_intensity,0);
        }
    }
}

void graspWidget::publishLinkColor(std::string link_name, unsigned char r, unsigned char g, unsigned char b)
{
    flor_ocs_msgs::OCSLinkColor cmd;

    cmd.link = link_name;
    cmd.r = r;
    cmd.g = g;
    cmd.b = b;

    hand_link_color_pub_.publish(cmd);
}

void graspWidget::publishHandPose(unsigned int id)
{
    //std::cout << "publishing hand pose for grasp id " << id << std::endl;
    //ROS_ERROR("publishing hand pose for grasp id %d",id);
    unsigned int grasp_index;
    for(grasp_index = 0; grasp_index < grasp_db_.size(); grasp_index++)
        if(grasp_db_[grasp_index].grasp_id == id)
            break;

    if(grasp_index == grasp_db_.size()) return;

    // get the selected grasp pose
    geometry_msgs::Pose grasp_transform;//geometry_msgs::PoseStamped grasp_transform;
    if(ui->show_grasp_radio->isChecked())
        grasp_transform = grasp_db_[grasp_index].final_pose;//grasp_transform.pose = grasp_db_[grasp_index].final_pose;
    else
        grasp_transform = grasp_db_[grasp_index].pre_grasp_pose;//grasp_transform.pose = grasp_db_[grasp_index].pre_grasp_pose;

    //ROS_ERROR("Grasp transform before: p=(%f, %f, %f) q=(%f, %f, %f, %f)",grasp_transform.position.x,grasp_transform.position.y,grasp_transform.position.z,grasp_transform.orientation.w,grasp_transform.orientation.x,grasp_transform.orientation.y,grasp_transform.orientation.z);

    // do the necessary transforms for graspit
    staticTransform(grasp_transform);//staticTransform(grasp_transform.pose);

    //ROS_ERROR("Grasp transform after:  p=(%f, %f, %f) q=(%f, %f, %f, %f)",grasp_transform.position.x,grasp_transform.position.y,grasp_transform.position.z,grasp_transform.orientation.w,grasp_transform.orientation.x,grasp_transform.orientation.y,grasp_transform.orientation.z);

    //grasp_transform.header.stamp = ros::Time(0);
    //grasp_transform.header.frame_id = (QString("/template_tf_")+QString::number(selected_template_id_)).toStdString();

    unsigned int template_index;
    for(template_index = 0; template_index < last_template_list_.template_id_list.size(); template_index++)
        if(last_template_list_.template_id_list[template_index] == selected_template_id_)
            break;

    if(template_index == last_template_list_.template_id_list.size()) return;

    geometry_msgs::PoseStamped template_transform;
    template_transform.pose = last_template_list_.pose[template_index].pose;
    //ROS_ERROR("Template transform:     p=(%f, %f, %f) q=(%f, %f, %f, %f)",template_transform.pose.position.x,template_transform.pose.position.y,template_transform.pose.position.z,template_transform.pose.orientation.w,template_transform.pose.orientation.x,template_transform.pose.orientation.y,template_transform.pose.orientation.z);

    geometry_msgs::PoseStamped hand_transform;
    calcWristTarget(grasp_transform, template_transform, hand_transform);
    //ROS_ERROR("Hand transform:         p=(%f, %f, %f) q=(%f, %f, %f, %f)",hand_transform.pose.position.x,hand_transform.pose.position.y,hand_transform.pose.position.z,hand_transform.pose.orientation.w,hand_transform.pose.orientation.x,hand_transform.pose.orientation.y,hand_transform.pose.orientation.z);

    hand_transform.header.stamp = ros::Time::now();
    hand_transform.header.frame_id = "/world";

    //try
    {
        // transform to world
        //tf_.transformPose("/world", grasp_transform, hand_transform);

        // publish
        ghost_hand_pub_.publish(hand_transform);

        publishHandJointStates(grasp_index);
    }
    /*catch(tf::TransformException e)
    {
        ROS_ERROR("Tf Pose Republisher: Transform from %s to %s failed: %s \n", grasp_transform.header.frame_id.c_str(), "/world", e.what() );
    }*/
}

void graspWidget::publishHandJointStates(unsigned int grasp_index)
{
    if(hand_type == "irobot")
    {
        sensor_msgs::JointState joint_states;

        joint_states.header.stamp = ros::Time::now();
        joint_states.header.frame_id = std::string("/")+hand+std::string("_hand_model/")+hand+"_palm";

        // must match the order used in the .grasp file
        joint_states.name.push_back(hand+"_f0_j1");
        joint_states.name.push_back(hand+"_f1_j1");
        joint_states.name.push_back(hand+"_f2_j1");
        joint_states.name.push_back(hand+"_f0_j0"); // .grasp finger position [4] -> IGNORE [3], use [4] for both
        joint_states.name.push_back(hand+"_f1_j0"); // .grasp finger position [4]
        joint_states.name.push_back(hand+"_f0_j3"); // 0 for now
        joint_states.name.push_back(hand+"_f1_j3"); // 0 for now
        joint_states.name.push_back(hand+"_f2_j3"); // 0 for now

        joint_states.position.resize(joint_states.name.size());
        joint_states.effort.resize(joint_states.name.size());
        joint_states.velocity.resize(joint_states.name.size());

        for(unsigned int i = 0; i < joint_states.position.size(); ++i)
        {
            joint_states.effort[i] = 0;
            joint_states.velocity[i] = 0;
            if(grasp_index == -1 || ui->show_pre_grasp_radio->isChecked())
                joint_states.position[i] = 0;
            else
                joint_states.position[i] = grasp_db_[grasp_index].finger_joints[i];
            //ROS_ERROR("Setting Finger Joint %s to %f",joint_states.name[i].c_str(),joint_states.position[i]);
        }

        ghost_hand_joint_state_pub_.publish(joint_states);
    }
    else
    {
        sensor_msgs::JointState joint_states;

        joint_states.header.stamp = ros::Time::now();
        joint_states.header.frame_id = std::string("/")+hand+std::string("_hand_model/")+hand+"_palm";

        // must match those inside of the /sandia_hands/?_hand/joint_states/[right_/left_]+
        joint_states.name.push_back(hand+"_f0_j0");
        joint_states.name.push_back(hand+"_f0_j1");
        joint_states.name.push_back(hand+"_f0_j2");
        joint_states.name.push_back(hand+"_f1_j0");
        joint_states.name.push_back(hand+"_f1_j1");
        joint_states.name.push_back(hand+"_f1_j2");
        joint_states.name.push_back(hand+"_f2_j0");
        joint_states.name.push_back(hand+"_f2_j1");
        joint_states.name.push_back(hand+"_f2_j2");
        joint_states.name.push_back(hand+"_f3_j0");
        joint_states.name.push_back(hand+"_f3_j1");
        joint_states.name.push_back(hand+"_f3_j2");

        joint_states.position.resize(joint_states.name.size());
        joint_states.effort.resize(joint_states.name.size());
        joint_states.velocity.resize(joint_states.name.size());

        for(unsigned int i = 0; i < joint_states.position.size(); ++i)
        {
            joint_states.effort[i] = 0;
            joint_states.velocity[i] = 0;
            if(grasp_index == -1)
                joint_states.position[i] = 0;
            else
                joint_states.position[i] = grasp_db_[grasp_index].finger_joints[i];
            //ROS_ERROR("Setting Finger Joint %s to %f",joint_states.name[i].c_str(),joint_states.position[i]);
        }

        ghost_hand_joint_state_pub_.publish(joint_states);
    }
}

// assume this function is called within mutex block
int graspWidget::calcWristTarget(const geometry_msgs::Pose& wrist_pose,const geometry_msgs::PoseStamped& template_pose, geometry_msgs::PoseStamped& final_pose)
{
    // Transform wrist_pose into the template pose frame
    //   @TODO        "wrist_target_pose.pose   = T(template_pose)*wrist_pose";
    tf::Transform wt_pose;
    tf::Transform tp_pose;
    tf::Transform target_pose;

    wt_pose.setRotation(tf::Quaternion(wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z,wrist_pose.orientation.w));
    wt_pose.setOrigin(tf::Vector3(wrist_pose.position.x,wrist_pose.position.y,wrist_pose.position.z) );
    tp_pose.setRotation(tf::Quaternion(template_pose.pose.orientation.x,template_pose.pose.orientation.y,template_pose.pose.orientation.z,template_pose.pose.orientation.w));
    tp_pose.setOrigin(tf::Vector3(template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z) );

    target_pose = tp_pose * wt_pose;  //I assume this works

    tf::Quaternion tg_quat;
    tf::Vector3    tg_vector;
    tg_quat   = target_pose.getRotation();
    tg_vector = target_pose.getOrigin();

    final_pose.pose.orientation.w = tg_quat.getW();
    final_pose.pose.orientation.x = tg_quat.getX();
    final_pose.pose.orientation.y = tg_quat.getY();
    final_pose.pose.orientation.z = tg_quat.getZ();

    final_pose.pose.position.x = tg_vector.getX();
    final_pose.pose.position.y = tg_vector.getY();
    final_pose.pose.position.z = tg_vector.getZ();

//        ROS_ERROR(" %s wrist: frame=%s p=(%f, %f, %f) q=(%f, %f, %f, %f)",
//                 hand.c_str(), "wrist",
//                 wrist_pose.position.x,wrist_pose.position.y,wrist_pose.position.z,
//                 wrist_pose.orientation.w, wrist_pose.orientation.x, wrist_pose.orientation.y, wrist_pose.orientation.z);
//        ROS_ERROR("ghost_hand_pub_.publish(hand_transform); %s template: frame=%s p=(%f, %f, %f) q=(%f, %f, %f, %f)",
//                 hand.c_str(), template_pose.header.frame_id.c_str(),
//                 template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z,
//                 template_pose.pose.orientation.w, template_pose.pose.orientation.x, template_pose.pose.orientation.y, template_pose.pose.orientation.z);
//        ROS_ERROR(" %s target: frame=%s p=(%f, %f, %f) q=(%f, %f, %f, %f)",
//                 hand.c_str(), final_pose.header.frame_id.c_str(),
//                 final_pose.pose.position.x,final_pose.pose.position.y,final_pose.pose.position.z,
//                 final_pose.pose.orientation.w, final_pose.pose.orientation.x, final_pose.pose.orientation.y, final_pose.pose.orientation.z);
    return 0;
}

int graspWidget::staticTransform(geometry_msgs::Pose& palm_pose)
{
    tf::Transform o_T_hand;    //describes hand in object's frame
    tf::Transform o_T_pg;       //describes palm_from_graspit in object's frame
    tf::Transform pg_T_rhand;   //describes r_hand in palm_from_graspit frame
    tf::Transform pg_T_lhand;   //describes l_hand in palm_from_graspit frame


    o_T_pg.setRotation(tf::Quaternion(palm_pose.orientation.x,palm_pose.orientation.y,palm_pose.orientation.z,palm_pose.orientation.w));
    o_T_pg.setOrigin(tf::Vector3(palm_pose.position.x,palm_pose.position.y,palm_pose.position.z) );

    if(hand_type == "irobot")
    {

        pg_T_rhand = tf::Transform(tf::Matrix3x3(1,0,0,0,1,0,0,0,1),tf::Vector3(0.0,0.0,0.0)); // but we need to got to right_palm
        pg_T_lhand = tf::Transform(tf::Matrix3x3(-1,0,0,0,-1,0,0,0,1),tf::Vector3(0.0,0.0,0.0)); // but we need to got to left_palm
    }
    else
    {
        pg_T_rhand = tf::Transform(tf::Matrix3x3(0,-1,0, 1,0,0,0,0,1),tf::Vector3(0.0173,-0.0587,-0.0061)); // but we need to got to right_palm
        pg_T_rhand.inverse();

        tf::Quaternion left_quat = pg_T_rhand.getRotation();
        //left_quat.setW(-left_quat.w());
        left_quat.setX(-left_quat.x());
        tf::Vector3 left_pos = pg_T_rhand.getOrigin();
        left_pos.setX(-left_pos.x());
        pg_T_lhand = tf::Transform(left_quat,left_pos); // but we need to got to left_palm
    }

    if(hand == "right")
    {
        o_T_hand = o_T_pg * pg_T_rhand;
    }
    else
    {
        o_T_hand = o_T_pg * pg_T_lhand;
    }

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

int graspWidget::hideHand()
{
    geometry_msgs::PoseStamped hand_transform;
    hand_transform.pose.position.z = 10000;
    hand_transform.pose.orientation.w = 1;
    hand_transform.header.stamp = ros::Time::now();
    hand_transform.header.frame_id = "/world";
    ghost_hand_pub_.publish(hand_transform);

    publishHandJointStates(-1);
}

void graspWidget::on_show_grasp_toggled(bool checked)
{
    show_grasp_ = checked;

    ui->show_grasp_radio->setEnabled(show_grasp_);
    ui->show_pre_grasp_radio->setEnabled(show_grasp_);
}
