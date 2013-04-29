#include "graspWidget.h"
#include "ui_graspWidget.h"
#include <ros/package.h>
#include <QColor>
#include <QProgressBar>
#include <QSlider>
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"

//grasp_testing grasp_testing_simple.launch

graspWidget::graspWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::graspWidget)
{
    ui->setupUi(this);
    ui->templateBox->setDisabled(true);
    ui->graspBox->setDisabled(true);
    ui->performButton->setDisabled(true);
    ui->templateButton->setDisabled(true);
    templateMatchDone = false;
    render_panel_ = new rviz::RenderPanel();
    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

    std::string templatePath = (ros::package::getPath("templates"))+"/";//vigir_grasp_control") + "/../templates/";
    std::cout << "--------------<" << templatePath << ">\n" << std::endl;
    template_dir_path_ = QString(templatePath.c_str());
    template_id_db_path_ = template_dir_path_+QString("grasp_templates.txt");
    grasp_db_path_ = template_dir_path_+QString("grasp_library.grasp");
    std::string temp = "";
    currentGraspMode = 0;
    QLabel foo;
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    nhp.param<std::string>("graspWidget/hand",temp,"default");
    if(temp == "left")
        hand = "left";
    else
        hand = "right";
    ROS_INFO("  Grasp widget using %s hand (%s)",hand.c_str(), temp.c_str());

    initTemplateIdMap();
    initGraspDB();

    template_list_sub_ = nh_.subscribe<flor_ocs_msgs::OCSTemplateList>( "/template/list", 5, &graspWidget::processTemplateList, this );

    //nh.getParam("/vigir_grasp_control/templates",templatePath);
    grasp_state_sub_ = nh_.subscribe<flor_grasp_msgs::GraspState>( "/template/active_state", 1, &graspWidget::graspStateRecieved, this );

    //grasp_selected_sub_ = nh_.subscribe<flor_grasp_msgs::GraspSelection>( "/template/grasp_selected", 1, &graspWidget::graspSelectedRecieved, this );

    grasp_selection_pub_ = nh_.advertise<flor_grasp_msgs::GraspSelection>( "/template/grasp_selection", 1,false);
    grasp_release_pub_  = nh_.advertise<flor_grasp_msgs::GraspSelection>( "/template/release_grasp" , 1,false);

    grasp_mode_command_pub_ = nh.advertise<flor_grasp_msgs::GraspState>("/template/grasp_mode_command", 1, false);

    // advertise the template match button pressed
    template_match_request_pub_ = nh_.advertise<flor_grasp_msgs::TemplateSelection>( "/template/template_match_request", 1, false );

    template_match_feedback_sub_ = nh_.subscribe<flor_grasp_msgs::TemplateSelection>( "/template/template_match_feedback", 1, &graspWidget::templateMatchFeedback, this );

    // and advertise the template update to update the manipulator
    //template_remove_pub_ = nh_.advertise<flor_ocs_msgs::OCSTemplateRemove>( "/template/remove", 1, false );

    // advertise the grasp selection
    //grasp_request_pub_ = nh_.advertise<flor_grasp_msgs::GraspSelection>( "/template/grasp_request", 1, false );
}
//SetStylesheet to change on the fly

graspWidget::~graspWidget()
{
    delete ui;
}

void graspWidget::templateMatchFeedback (const flor_grasp_msgs::TemplateSelection::ConstPtr& feedback)
{
    QPalette pal = ui->templateButton->palette();
    if(feedback->confidence.data >= 85)
        pal.setColor(QPalette::Button,Qt::green);
    else if(feedback->confidence.data >=60)
        pal.setColor(QPalette::Button,Qt::yellow);
    else
        pal.setColor(QPalette::Button,Qt::red);
    ui->templateButton->setPalette(pal);
    ui->templateButton->setAutoFillBackground(true);
    std::cout << "Template confidence recieved and found to be " << (int)feedback->confidence.data << std::endl;
    templateMatchDone = true;
}

void graspWidget::graspStateRecieved (const flor_grasp_msgs::GraspState::ConstPtr& graspState)
{
    std::cout << "Grasp State message recieved" << graspState << std::endl;
    uint8_t mode  = (graspState->grasp_state.data&0xF0) >> 4;
    uint8_t state = graspState->grasp_state.data&0x0F;
    setProgressLevel(graspState->grip.data);

    ui->userSlider->setValue(graspState->grip.data);
    std::cout << "     mode=" << uint32_t(mode) << "   state="<< uint32_t(state) << std::endl;
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
    std::cout << "Template list recieved containing " << list->template_id_list.size() << " elements" << std::cout;
    lastList = *list;
    if(list->template_list.size() > 0)
    {
        ui->templateBox->setDisabled(false);
        ui->graspBox->setDisabled(false);
        ui->templateButton->setDisabled(false);
        ui->performButton->setDisabled(false);
    }
    for(int i=0; i<list->template_list.size();i++)
    {
        std::string templateName = list->template_list[i];
        if(templateName.size() > 5 && templateName.substr(templateName.size()-5,5) == ".mesh")
            templateName = templateName.substr(0,templateName.size()-5);
        std::cout << "template item " << (int)list->template_id_list[i] << " has name " << templateName << std::endl;
        if(ui->templateBox->count() < i+1)
        {
            ui->templateBox->addItem(QString::fromStdString(templateName));
        }
            else if( ui->templateBox->itemText(i).toStdString() != templateName)
        {
            ui->templateBox->setItemText(i,QString::fromStdString(templateName));
        }
    }
}

void graspWidget::initTemplateMode()
{
    if(lastList.template_id_list.size() > 0)
    {
        ui->templateBox->setDisabled(false);
        ui->graspBox->setDisabled(false);
    }
}

//currentStateLabel
void graspWidget::setProgressLevel(uint8_t level)
{
    std::cout << "setting fill level to be " << (int)level << std::endl;
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
        //std::cout << "initial grasp type: " << grasp.initial_grasp_type << std::endl;

        //std::cout << "finger joints: ";
        for(int j = 5; j < 17; j++)
        {
            grasp.finger_joints[j-5] = db[i][j].toFloat(&ok);
            //std::cout << grasp.finger_joints[j-5] << ",";
        }
        //std::cout << std::endl;

        grasp.final_pose.position.x = db[i][18].toFloat(&ok);
        grasp.final_pose.position.y = db[i][19].toFloat(&ok);
        grasp.final_pose.position.z = db[i][20].toFloat(&ok);
        grasp.final_pose.orientation.x = db[i][21].toFloat(&ok);
        grasp.final_pose.orientation.y = db[i][22].toFloat(&ok);
        grasp.final_pose.orientation.z = db[i][23].toFloat(&ok);
        grasp.final_pose.orientation.w = db[i][24].toFloat(&ok);
        //std::cout << "final pose: " << grasp.final_pose.position.x << ", " << grasp.final_pose.position.y << ", " << grasp.final_pose.position.z << ", " <<
        //             grasp.final_pose.orientation.x << ", " << grasp.final_pose.orientation.y << ", " << grasp.final_pose.orientation.y << ", " << grasp.final_pose.orientation.w << std::endl;

        grasp.pre_grasp_pose.position.x = db[i][26].toFloat(&ok);
        grasp.pre_grasp_pose.position.y = db[i][27].toFloat(&ok);
        grasp.pre_grasp_pose.position.z = db[i][28].toFloat(&ok);
        grasp.pre_grasp_pose.orientation.x = db[i][29].toFloat(&ok);
        grasp.pre_grasp_pose.orientation.y = db[i][30].toFloat(&ok);
        grasp.pre_grasp_pose.orientation.z = db[i][31].toFloat(&ok);
        grasp.pre_grasp_pose.orientation.w = db[i][32].toFloat(&ok);
        //std::cout << "final pose: " << grasp.pre_grasp_pose.position.x << ", " << grasp.pre_grasp_pose.position.y << ", " << grasp.pre_grasp_pose.position.z << ", " <<
        //             grasp.pre_grasp_pose.orientation.x << ", " << grasp.pre_grasp_pose.orientation.y << ", " << grasp.pre_grasp_pose.orientation.y << ", " << grasp.pre_grasp_pose.orientation.w << std::endl;

        grasp_db_.push_back(grasp);
    }
}


void graspWidget::sendManualMsg(uint8_t level)
{
    flor_grasp_msgs::GraspState cmd;
    cmd.grip.data = level;
    cmd.grasp_state.data = (flor_grasp_msgs::GraspState::MANUAL_GRASP_MODE)<<4;
    grasp_mode_command_pub_.publish(cmd);
    std::cout << "Sent command message as manual grip to" << (int)level << std::endl;
}

void graspWidget::on_userSlider_sliderReleased()
{
    if(ui->manualRadio->isChecked())
    {
        setProgressLevel(ui->userSlider->value());
        sendManualMsg(ui->userSlider->value());
    }
    else if(ui->userSlider->value() > 100)
    {
        flor_grasp_msgs::GraspState msg;//68 mode
        msg.grasp_state.data = 68;
        msg.grip.data = ui->userSlider->value();
        grasp_mode_command_pub_.publish(msg);
    }
    else
    {
        std::cout << "slider changed while not in manual mode. New position is " << ui->userSlider->value() << std::endl;
    }
}

void graspWidget::on_releaseButton_clicked()
{
    std::cout << "release the grasp requested" << std::endl;
    ui->userSlider->setValue(0);
    flor_grasp_msgs::GraspState msg;
    msg.grasp_state.data = 0;
    msg.grip.data = 0;
    grasp_mode_command_pub_.publish(msg);

    flor_grasp_msgs::GraspSelection grasp_msg;
    grasp_msg.header.stamp=ros::Time::now();
    grasp_msg.grasp_id.data      = 0;
    grasp_msg.template_id.data   = 0;
    grasp_msg.template_type.data = 0;
    grasp_release_pub_.publish(grasp_msg);
}

void graspWidget::on_templateButton_clicked()
{
    std::cout << "template match requested..." << std::endl;
    flor_grasp_msgs::TemplateSelection msg;
    int graspID = ui->graspBox->currentText().toInt();
    for(int index = 0; index < grasp_db_.size(); index++)
    {
        if(grasp_db_[index].grasp_id == graspID)
            msg.template_type.data = grasp_db_[index].template_type;
    }
    msg.template_id.data = ui->templateBox->currentIndex();
    msg.pose.pose = lastList.pose[ui->templateBox->currentIndex()].pose;
    template_match_request_pub_.publish(msg);
}

void graspWidget::on_performButton_clicked()
{
    std::cout << "Performing grasp" << std::endl;
    flor_grasp_msgs::GraspSelection msg;
    msg.header.frame_id = "/word";
    int graspID = ui->graspBox->currentText().toInt();
    msg.grasp_id.data = graspID;
    msg.template_id.data = ui->templateBox->currentIndex();
    for(int index = 0; index < grasp_db_.size(); index++)
    {
        if(grasp_db_[index].grasp_id == graspID)
            msg.template_type.data = grasp_db_[index].template_type;
    }
    grasp_selection_pub_.publish(msg);
}

void graspWidget::on_templateBox_activated(const QString &arg1)
{
    std::cout << "updating the grasp widget box contents" << std::endl;
    while(ui->graspBox->count() >0)
        ui->graspBox->removeItem(0);
    for(int index=0; index < grasp_db_.size(); index++)
    {
        std::cout << "comparing db " << grasp_db_[index].template_name << " to " << arg1.toStdString() << std::endl;

        if(grasp_db_[index].template_name == arg1.toStdString() && grasp_db_[index].hand == hand)
        {
            std::cout << "Found grasp for template" << std::endl;
            ui->graspBox->addItem(QString::number(grasp_db_[index].grasp_id));
        }
    }
}

void graspWidget::on_templateRadio_clicked()
{
    flor_grasp_msgs::GraspState msg;
    msg.grasp_state.data = (flor_grasp_msgs::GraspState::TEMPLATE_GRASP_MODE)<<4;
    msg.grip.data = 0;
    grasp_mode_command_pub_.publish(msg);
    std::cout << "Sent Template mode message ("<< uint32_t(msg.grasp_state.data) << ") with " <<  uint32_t(msg.grip.data) << " manual grip level to " << hand << " hand" << std::endl;

}

void graspWidget::on_manualRadio_clicked()
{
    flor_grasp_msgs::GraspState msg;
    msg.grasp_state.data = (flor_grasp_msgs::GraspState::MANUAL_GRASP_MODE)<<4;
    msg.grip.data = ui->userSlider->value();
    grasp_mode_command_pub_.publish(msg);
    std::cout << "Sent Manual mode message ("<< uint32_t(msg.grasp_state.data) << ") with " <<  uint32_t(msg.grip.data) << " manual grip level to " << hand << " hand" << std::endl;

}
