#include <ros/package.h>

#include "image_manager_widget.h"
#include "ui_image_manager_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>
#include <QSignalMapper>



ImageManagerWidget::ImageManagerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ImageManagerWidget)
{    
    // Use "images" package.  @TODO make this a parameter
    std::string image_path = ros::package::getPath("images")+"/";//vigir_grasp_control") + "/../images/";
    std::cout << "--------------<" << image_path << ">\n" << std::endl;

    image_dir_path_ = QString(image_path.c_str());

    ui->setupUi(this);

    connect(ui->tableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(editSlot(int, int)));

    image_id_db_path_ = image_dir_path_+QString("grasp_images.txt");
    grasp_db_path_ = image_dir_path_+QString("grasp_library.grasp");

    initImageIdMap();
    initGraspDB();

    // subscribe to the topic to load all images
    image_list_sub_ = nh_.subscribe<flor_ocs_msgs::OCSImageList>( "/image/list", 5, &ImageManagerWidget::processImageList, this );

    // advertise the image match button pressed
    image_match_request_pub_ = nh_.advertise<flor_grasp_msgs::ImageSelection>( "/image/image_match_request", 1, false );

    // and advertise the image update to update the manipulator
    image_remove_pub_ = nh_.advertise<flor_ocs_msgs::OCSImageRemove>( "/image/remove", 1, false );

    // advertise the grasp selection
    grasp_request_pub_ = nh_.advertise<flor_grasp_msgs::GraspSelection>( "/image/grasp_request", 1, false );

    timer.start(33, this);
}

ImageManagerWidget::~ImageManagerWidget()
{
    delete ui;
}

void ImageManagerWidget::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

void ImageManagerWidget::initImageIdMap()
{
    std::vector< std::vector<QString> > db = readTextDBFile(image_id_db_path_);

    for(int i = 0; i < db.size(); i++)
    {
        bool ok;
        unsigned char id = db[i][0].toUInt(&ok, 10) & 0x000000ff;
        std::string image_path(db[i][1].toUtf8().constData());
        std::cout << "-> Adding image (" << image_path << ") to id (" << (unsigned int)id << ") map" << std::endl;
        image_id_map_.insert(std::pair<unsigned char,std::string>(id,image_path));
    }
}

void ImageManagerWidget::initGraspDB()
{
    std::vector< std::vector<QString> > db = readTextDBFile(grasp_db_path_);
    for(int i = 0; i < db.size(); i++)
    {
        bool ok;
        // [0] grasp id, [1] image type, [2] hand, [3] initial grasp type, [4] DISCARD, [5-16] finger joints (12), [17] DISCARD, [18-24] final grasp pose relative to image (x,y,z,qx,qy,qz,qw), [25] DISCARD, [26-32] pre-grasp pose relative to image (x,y,z,qx,qy,qz,qw)
        GraspDBItem grasp;
        //std::cout << "-> Adding grasp to grasp DB" << std::endl;
        grasp.grasp_id = db[i][0].toUInt(&ok, 10) & 0x0000ffff;
        //std::cout << "id: " << (unsigned int)grasp.grasp_id << std::endl;

        grasp.image_type = db[i][1].toUInt(&ok, 10) & 0x000000ff;
        //std::cout << "image type: " << (unsigned int)grasp.image_type << std::endl;

        grasp.image_name = image_id_map_.find(grasp.image_type)->second;
        //std::cout << "image name: " << grasp.image_name << std::endl;

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

// will return a vector with rows, each row containing a QStringList with all columns
std::vector< std::vector<QString> > ImageManagerWidget::readTextDBFile(QString path)
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

void ImageManagerWidget::processImageList(const flor_ocs_msgs::OCSImageList::ConstPtr& msg)
{
    // reset table
    ui->tableWidget->clearContents();
    ui->tableWidget->setRowCount(msg->image_list.size());

    // add info about images to the table
    QSignalMapper* signalMapper = new QSignalMapper(this);
    QTableWidgetItem * item;

    for(int i = 0; i < msg->image_list.size(); i++)
    {
        //std::cout << "Image: " << msg->image_list[i] << std::endl;

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
        item = new QTableWidgetItem(QString::number(msg->image_id_list[i]));
        ui->tableWidget->setItem(i,1,item);
        item = new QTableWidgetItem(QString::fromUtf8(msg->image_list[i].substr(0,msg->image_list[i].size()-5).c_str()));
        ui->tableWidget->setItem(i,2,item);
        item = new QTableWidgetItem(QString::number(px)+", "+QString::number(py)+", "+QString::number(pz));
        ui->tableWidget->setItem(i,3,item);
        item = new QTableWidgetItem(QString::number(qx)+", "+QString::number(qy)+", "+QString::number(qz)+", "+QString::number(qw));
        ui->tableWidget->setItem(i,4,item);
        item = new QTableWidgetItem(QString::fromUtf8(msg->pose[i].header.frame_id.c_str()));
        ui->tableWidget->setItem(i,5,item);
        QComboBox *combo = new QComboBox();
        configureGrasps(msg->image_list[i].substr(0,msg->image_list[i].size()-5), combo);
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

void ImageManagerWidget::configureGrasps(std::string image_name, QComboBox* combo_box)
{
    //std::cout << "looking for grasps for image " << image_name << std::endl;
    // add all grasp ids to the combo box
    for(int i = 0; i < grasp_db_.size(); i++)
    {
        if(image_name.compare(grasp_db_[i].image_name) == 0)
        {
            //std::cout << "  found " << (unsigned int)grasp_db_[i].grasp_id << std::endl;
            combo_box->addItem(QString::number((unsigned int)grasp_db_[i].grasp_id));
        }
    }
}

void ImageManagerWidget::removeImage(int id)
{
    flor_ocs_msgs::OCSImageRemove cmd;

    cmd.image_id = id;

    // publish image to be removed
    image_remove_pub_.publish( cmd );
}

void ImageManagerWidget::editSlot(int row, int col)
{
    if(col == 0)
    {
        removeImage(ui->tableWidget->item(row,1)->text().toUInt() & 0x000000ff);
    }
    else if(col == 7)
    {
        flor_grasp_msgs::ImageSelection cmd;

        cmd.image_id.data = ui->tableWidget->item(row,1)->text().toUInt() & 0x000000ff;
        std::string image_name = ui->tableWidget->item(row,2)->text().toUtf8().constData();

        for(int i = 0; i < grasp_db_.size(); i++)
        {
            if(grasp_db_[i].image_name.compare(image_name) == 0)
            {
                cmd.image_type.data = grasp_db_[i].image_type;
                break;
            }
        }
        cmd.pose.header.frame_id = "/world";

        // publish image to be matched
        image_match_request_pub_.publish( cmd );
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
                cmd.image_id.data = ui->tableWidget->item(row,1)->text().toUInt() & 0x000000ff;
                cmd.image_type.data = grasp_db_[i].image_type;
                cmd.header.frame_id = "/world";

                // publish image to be matched
                grasp_request_pub_.publish( cmd );

                break;
            }
        }
    }
}
