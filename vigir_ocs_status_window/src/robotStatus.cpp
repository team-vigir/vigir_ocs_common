#include "robotStatus.h"
#include <QVBoxLayout>
#include <ros/ros.h>
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"

robotStatus::robotStatus(QWidget *parent) :
    QWidget(parent)
{
    this->setMinimumSize(400,400);

    render_panel_ = new rviz::RenderPanel();
    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

    bold.setBold(true);
    normal.setBold(false);
    msgTable = new QTableWidget();
    msgTable->setColumnCount(2);
    clearButton = new QPushButton();
    clearButton->setText("Clear Table Contents");
    QVBoxLayout* main_layout = new QVBoxLayout;
    main_layout->addWidget(msgTable);
    main_layout->addWidget(clearButton);
    //std::cout << "Adding layout..." << std::endl;
    setLayout(main_layout);
    msgTable->setColumnWidth(0,70);
    msgTable->setColumnWidth(1,290);
    //labels = new QStringList();
    labels.push_back("Sim Time");
    labels.push_back("Message Contents");
    msgTable->setHorizontalHeaderLabels(labels);
    unreadMsgs=0;
    numError = 0;
    numWarn = 0;
    ros::NodeHandle nh;
    std::string fileName;
    if(nh.getParam("robotErrorFileLocation",fileName))
            messagesFile.setFileName(fileName.c_str());
    else
            messagesFile.setFileName("/home/messages.csv");
    loadFile();
    rosSubscriber = nh.subscribe<flor_ocs_msgs::OCSRobotError>( "/atlas/robot_error", 2, &robotStatus::recievedMessage, this );
    std::cout << "Done setting up waiting for messages." << std::endl;
    ros::spinOnce();
    clearButton->connect(clearButton,SIGNAL(clicked()),this,SLOT(on_clearButton_clicked()));
    msgTable->connect(msgTable,SIGNAL(cellClicked(int,int)),this,SLOT(on_msgTable_cellClicked(int,int)));
}

void robotStatus::recievedMessage(const flor_ocs_msgs::OCSRobotError::ConstPtr& msg)
{

    int color = msg->msg >> 6;
    int msgNum = msg->msg - color*64;
    std::cout << "Recieved message. color = " << color << "msgNum = " << msgNum << std::endl;
    QTableWidgetItem* text = new QTableWidgetItem();
    QTableWidgetItem* time = new QTableWidgetItem();
    ros::Time curTime = ros::Time::now();
    time->setText(QString::number(curTime.toSec()));
    if(color == 1)
    {
        text->setBackgroundColor(Qt::yellow);
        time->setBackgroundColor(Qt::yellow);
        numWarn++;
    }
    else if(color == 3)
    {
        text->setBackgroundColor(Qt::red);
        time->setBackgroundColor(Qt::red);
        numError++;
    }

    if(msgNum >= errors.count()/2 && errors.count() != 0)
    {
        text->setText("Message number recieved not in messages list");
        text->setBackgroundColor(Qt::red);
        time->setBackgroundColor(Qt::red);
        numError++;
    }
    else if(errors.count() > 0)
        text->setText(errors.at(msgNum*2+1));
    else
    {
        text->setText("Cannot find message due to empty or missing data file");
        text->setBackground(Qt::red);
        time->setBackgroundColor(Qt::red);
        numError++;
    }

    time->setFont(bold);
    text->setFont(bold);
    msgTable->insertRow(msgTable->rowCount());
    msgTable->setItem(msgTable->rowCount()-1,0,time);
    msgTable->setItem(msgTable->rowCount()-1,1,text);
    messages.push_back(time);
    messages.push_back(text);
    unreadMsgs++;
}

void robotStatus::loadFile()
{
    std::cout << "Reading in csv File for error list at " << messagesFile.fileName().toStdString() << std::endl;
    if(messagesFile.open(QIODevice::ReadOnly))
    {
        QString data;
        data = messagesFile.readAll();
        errors = data.split(',');
        messagesFile.close();
        std::cout << "Done reading in file" << std::endl;
    }
    else
        std::cout << "Failed to read in file" << std::endl;
 }

void robotStatus::on_clearButton_clicked()
{
    std::cout << "Clear button pressed..." << std::endl;
    //msgTable = new QTableWidget();
    //msgTable->setColumnCount(2);
    //msgTable->setColumnWidth(0,50);
    //msgTable->setHorizontalHeaderLabels(labels);
    msgTable->clearContents();
    while(msgTable->rowCount() > 0)
        msgTable->removeRow(0);
    messages.clear();
    unreadMsgs=0;
    numError = 0;
    numWarn = 0;
}

void robotStatus::on_msgTable_cellClicked(int row, int column)
{
    std::cout << "row " << row << " clicked to clear" <<std::endl;
    (messages[(row)*2])->setFont(normal);
    (messages[(row)*2+1])->setFont(normal);
}

int robotStatus::getNumUnread()
{
    return unreadMsgs;
}
int robotStatus::getNumError()
{
    return numError;
}
int robotStatus::getNumWarn()
{
    return numWarn;
}

robotStatus::~robotStatus()
{
    //delete ui;
}
