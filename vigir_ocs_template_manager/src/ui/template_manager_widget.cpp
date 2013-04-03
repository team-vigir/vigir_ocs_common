#include <ros/package.h>

#include "template_manager_widget.h"
#include "ui_template_manager_widget.h"
#include "stdio.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include <iostream>
#include <QPainter>
#include <QtGui>
#include <QSignalMapper>



TemplateManagerWidget::TemplateManagerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TemplateManagerWidget)
{    
    // Use "templates" package.  @TODO make this a parameter
    std::string template_path = ros::package::getPath("templates");//vigir_grasp_control") + "/../templates/";
    std::cout << "--------------<" << template_path << ">\n" << std::endl;

    template_dir_path_ = QString(template_path.c_str());

    ui->setupUi(this);

    render_panel_ = new rviz::RenderPanel();
    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

    connect(ui->tableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(editSlot(int, int)));

    template_id_db_path_ = template_dir_path_+QString("grasp_templates.txt");
    grasp_db_path_ = template_dir_path_+QString("grasp_library.grasp");

    initTemplateIdMap();
    initGraspDB();

    // subscribe to the topic to load all templates
    template_list_sub_ = nh_.subscribe<flor_ocs_msgs::OCSTemplateList>( "/template/list", 5, &TemplateManagerWidget::processTemplateList, this );

    // advertise the template match button pressed
    template_match_request_pub_ = nh_.advertise<flor_grasp_msgs::TemplateSelection>( "/template/template_match_request", 1, false );

    // and advertise the template update to update the manipulator
    template_remove_pub_ = nh_.advertise<flor_ocs_msgs::OCSTemplateRemove>( "/template/remove", 1, false );

    // advertise the grasp selection
    grasp_request_pub_ = nh_.advertise<flor_grasp_msgs::GraspSelection>( "/template/grasp_request", 1, false );
}

TemplateManagerWidget::~TemplateManagerWidget()
{
    delete ui;
}

void TemplateManagerWidget::initTemplateIdMap()
{
    std::vector< std::vector<QString> > db = readTextDBFile(template_id_db_path_);

    for(int i = 0; i < db.size(); i++)
    {
        bool ok;
        unsigned char id = db[i][0].toUInt(&ok, 10) & 0x000000ff;
        std::string template_path(db[i][1].toUtf8().constData());
        std::cout << "-> Adding template (" << template_path << ") to id (" << (unsigned int)id << ") map" << std::endl;
        template_id_map_.insert(std::pair<unsigned char,std::string>(id,template_path));
    }
}

void TemplateManagerWidget::initGraspDB()
{
    std::vector< std::vector<QString> > db = readTextDBFile(grasp_db_path_);
    for(int i = 0; i < db.size(); i++)
    {
        bool ok;
        // [0] grasp id, [1] template type, [2] hand, [3] initial grasp type, [4] DISCARD, [5-16] finger joints (12), [17] DISCARD, [18-24] final grasp pose relative to template (x,y,z,qx,qy,qz,qw), [25] DISCARD, [26-32] pre-grasp pose relative to template (x,y,z,qx,qy,qz,qw)
        GraspDBItem grasp;
        //std::cout << "-> Adding grasp to grasp DB" << std::endl;
        grasp.grasp_id = db[i][0].toUInt(&ok, 10) & 0x0000ffff;
        //std::cout << "id: " << (unsigned int)grasp.grasp_id << std::endl;

        grasp.template_type = db[i][1].toUInt(&ok, 10) & 0x000000ff;
        //std::cout << "template type: " << (unsigned int)grasp.template_type << std::endl;

        grasp.template_name = template_id_map_.find(grasp.template_type)->second;
        //std::cout << "template name: " << grasp.template_name << std::endl;

        grasp.hand = db[i][2].toUtf8().constData();
        //std::cout << "hand: " << grasp.hand << std::endl;

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

// will return a vector with rows, each row containing a QStringList with all columns
std::vector< std::vector<QString> > TemplateManagerWidget::readTextDBFile(QString path)
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

void TemplateManagerWidget::processTemplateList(const flor_ocs_msgs::OCSTemplateList::ConstPtr& msg)
{
    // reset table
    ui->tableWidget->clearContents();
    ui->tableWidget->setRowCount(msg->template_list.size());

    // add info about templates to the table
    QSignalMapper* signalMapper = new QSignalMapper(this);
    QTableWidgetItem * item;

    for(int i = 0; i < msg->template_list.size(); i++)
    {
        //std::cout << "Template: " << msg->template_list[i] << std::endl;

        float px,py,pz;
        px = msg->pose[i].pose.position.x;
        py = msg->pose[i].pose.position.y;
        pz = msg->pose[i].pose.position.z;
        float qx,qy,qz,qw;
        qw= msg->pose[i].pose.orientation.w;
        qx= msg->pose[i].pose.orientation.x;
        qy= msg->pose[i].pose.orientation.y;
        qz= msg->pose[i].pose.orientation.z;

        // remove   id   name   position   orientation   parentframe   grasp   confirmgrasp
        item = new QTableWidgetItem(QString("REMOVE"));
        item->setBackground(QBrush(QColor(200,200,200)));
        item->setForeground(QBrush(QColor(20,20,20)));
        item->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable);
        ui->tableWidget->setItem(i,0,item);
        item = new QTableWidgetItem(QString::number(msg->template_id_list[i]));
        ui->tableWidget->setItem(i,1,item);
        item = new QTableWidgetItem(QString::fromUtf8(msg->template_list[i].substr(0,msg->template_list[i].size()-5).c_str()));
        ui->tableWidget->setItem(i,2,item);
        item = new QTableWidgetItem(QString::number(px)+", "+QString::number(py)+", "+QString::number(pz));
        ui->tableWidget->setItem(i,3,item);
        item = new QTableWidgetItem(QString::number(qx)+", "+QString::number(qy)+", "+QString::number(qz)+", "+QString::number(qw));
        ui->tableWidget->setItem(i,4,item);
        item = new QTableWidgetItem(QString::fromUtf8(msg->pose[i].header.frame_id.c_str()));
        ui->tableWidget->setItem(i,5,item);
        QComboBox *combo = new QComboBox();
        configureGrasps(msg->template_list[i].substr(0,msg->template_list[i].size()-5), combo);
        // TODO: need to create changed function to keep track of what's being visualized
        //connect(combo, SIGNAL(currentIndexChanged(int)), signalMapper, SLOT(map()));
        //signalMapper->setMapping(combo, QString("%1").arg(i));
        ui->tableWidget->setCellWidget(i, 6, combo);
        //ui->tableWidget->setItem(i,6,item);
        item = new QTableWidgetItem(QString("MATCH"));
        item->setBackground(QBrush(QColor(200,200,200)));
        item->setForeground(QBrush(QColor(20,20,20)));
        item->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable);
        ui->tableWidget->setItem(i,7,item);
        item = new QTableWidgetItem(QString("GRASP"));
        item->setBackground(QBrush(QColor(200,200,200)));
        item->setForeground(QBrush(QColor(20,20,20)));
        item->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable);
        ui->tableWidget->setItem(i,8,item);
    }
}

void TemplateManagerWidget::configureGrasps(std::string template_name, QComboBox* combo_box)
{
    //std::cout << "looking for grasps for template " << template_name << std::endl;
    // add all grasp ids to the combo box
    for(int i = 0; i < grasp_db_.size(); i++)
    {
        if(template_name.compare(grasp_db_[i].template_name) == 0)
        {
            //std::cout << "  found " << (unsigned int)grasp_db_[i].grasp_id << std::endl;
            combo_box->addItem(QString::number((unsigned int)grasp_db_[i].grasp_id));
        }
    }
}

void TemplateManagerWidget::removeTemplate(int id)
{
    flor_ocs_msgs::OCSTemplateRemove cmd;

    cmd.template_id = id;

    // publish template to be removed
    template_remove_pub_.publish( cmd );
}

void TemplateManagerWidget::editSlot(int row, int col)
{
    if(col == 0)
    {
        QString item = ui->tableWidget->item(row,1)->text();
        bool ok;
        int convert = item.toInt(&ok, 10);
        if(ok)
            removeTemplate(convert);
    }
    else if(col == 7)
    {
        flor_grasp_msgs::TemplateSelection cmd;

        cmd.template_id.data = ui->tableWidget->item(row,0)->text().toUInt() & 0x000000ff;
        std::string template_name = ui->tableWidget->item(row,2)->text().toUtf8().constData();
        int index = 0;
        for(int i = 0; i < grasp_db_.size(); i++)
        {
            if(grasp_db_[i].template_name.compare(template_name) == 0)
            {
                cmd.template_type.data = grasp_db_[i].template_type;
                break;
            }
        }
        cmd.pose.header.frame_id = "/world";

        // publish template to be matched
        template_match_request_pub_.publish( cmd );
    }
    else if(col == 8)
    {
        QComboBox* combo = (QComboBox*)ui->tableWidget->cellWidget(row, 6);
        unsigned short grasp_id = combo->itemText(combo->currentIndex()).toUInt() & 0x0000ffff;

        for(int i = 0; i < grasp_db_.size(); i++)
        {
            if(grasp_id == grasp_db_[i].grasp_id)
            {
                flor_grasp_msgs::GraspSelection cmd;

                cmd.grasp_id.data = grasp_id;
                cmd.template_id.data = ui->tableWidget->item(row,0)->text().toUInt() & 0x000000ff;
                cmd.template_type.data = grasp_db_[i].template_type;
                cmd.header.frame_id = "/world";

                // publish template to be matched
                grasp_request_pub_.publish( cmd );

                break;
            }
        }
    }
}
