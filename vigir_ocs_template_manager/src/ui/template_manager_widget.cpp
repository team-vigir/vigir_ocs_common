#include "template_manager_widget.h"
#include "ui_template_manager_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>

TemplateManagerWidget::TemplateManagerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TemplateManagerWidget),
    templateDirPath(QString("/opt/vigir/rosbuild_ws/vigir_control/vigir_grasping/templates/")) // should read this from file
{
    ui->setupUi(this);

    connect(ui->tableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(editSlot(int, int)));

    // subscribe to the topic to load all templates
    template_list_sub_ = nh_.subscribe<flor_ocs_msgs::OCSTemplateList>( "/template/list", 5, &TemplateManagerWidget::processTemplateList, this );

    // and advertise the template update to update the manipulator
    template_remove_pub_ = nh_.advertise<flor_ocs_msgs::OCSTemplateRemove>( "/template/remove", 1, false );
}

TemplateManagerWidget::~TemplateManagerWidget()
{
    delete ui;
}

void TemplateManagerWidget::processTemplateList(const flor_ocs_msgs::OCSTemplateList::ConstPtr& msg)
{
    // reset table
    ui->tableWidget->clearContents();
    ui->tableWidget->setRowCount(msg->template_list.size());

    // add info about templates to the table
    QTableWidgetItem * item;

    for(int i = 0; i < msg->template_list.size(); i++)
    {
        std::cout << "Template: " << msg->template_list[i] << std::endl;

        float px,py,pz;
        px = msg->pose[i].pose.position.x;
        py = msg->pose[i].pose.position.y;
        pz = msg->pose[i].pose.position.z;
        float qx,qy,qz,qw;
        qw= msg->pose[i].pose.orientation.w;
        qx= msg->pose[i].pose.orientation.x;
        qy= msg->pose[i].pose.orientation.y;
        qz= msg->pose[i].pose.orientation.z;

        // remove   id   name   position   orientation   parentframe   grasp
        item = new QTableWidgetItem(QString("REMOVE"));
        item->setBackground(QBrush(QColor(0,0,0)));
        item->setForeground(QBrush(QColor(200,20,20)));
        ui->tableWidget->setItem(i,0,item);
        item = new QTableWidgetItem(QString::number(i));
        ui->tableWidget->setItem(i,1,item);
        item = new QTableWidgetItem(QString::fromUtf8(msg->template_list[i].c_str()));
        ui->tableWidget->setItem(i,2,item);
        item = new QTableWidgetItem(QString::number(px)+", "+QString::number(py)+", "+QString::number(pz));
        ui->tableWidget->setItem(i,3,item);
        item = new QTableWidgetItem(QString::number(qx)+", "+QString::number(qy)+", "+QString::number(qz)+", "+QString::number(qw));
        ui->tableWidget->setItem(i,4,item);
        item = new QTableWidgetItem(QString::fromUtf8(msg->pose[i].header.frame_id.c_str()));
        ui->tableWidget->setItem(i,5,item);
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
        removeTemplate(row);
    }
}
