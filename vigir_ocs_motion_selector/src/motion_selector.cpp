#include "motion_selector.h"
#include "ui_motion_selector.h"
#include <flor_control_msgs/FlorExecuteMotionRequest.h>
#include <qfile.h>
#include <QTextStream>
#include <QTreeWidget>
#include <QRegExp>

motion_selector::motion_selector(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::motion_selector)
{
    ui->setupUi(this);
    sliderVal = 1.00;
    processTextFile(QString::fromStdString("/opt/vigir/catkin_ws/src/vigir_ocs_common/vigir_ocs_motion_selector/src/tree.txt"));
    //ros::
    std::cout << "File done processing now populatin the tree." << std::endl;
    populateTree();
    ros::NodeHandle nh;
    message_pub_ = nh.advertise<flor_control_msgs::FlorExecuteMotionRequest>( "/flor/motion_service/motion_command",1,false);
    timer.start(33, this);
}

void motion_selector::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}
void motion_selector::processTextFile(QString path)
{
    QFile file(path);
    std::cout << "Trying to open file at " << path.toStdString() << std::endl;
    if(file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        std::cout << "File opened successfully... now parsing." << std::endl;
        QTextStream in(&file);
        while(!in.atEnd())
        {
            QString line = in.readLine();
            if(line[0] != '#')
            {
                line.truncate(line.size()-1);
                Task newTask;
                //taskList.push_back(newTask);
                //std::cout << "line TSK = " << line.toStdString.() << std::endl;
                newTask.name = line;
                line = in.readLine();
                line.remove(QRegExp("[\t]*"));
                if(line[0] != '#')
                {
                    while(line[0] != '}')
                    {
                        line.truncate(line.size()-1);
                        //std::cout << "line GRP = " << line.toStdString() << std::endl;
                        Group newGroup;
                        //taskList[taskLisdoublet.size()].groups.push_back(newGroup);
                        //taskList[taskList.size()].groups[taskList[taskList.size()].groups.size()].name = line;
                        newGroup.name = line;
                        line = in.readLine();
                        line.remove(QRegExp("[\t]*"));
                        if(line[0] != '#')
                        {
                            while(line[0] != '}')
                            {
                                //std::cout << "line PRM = " << line.toStdString() << std::endl;
                                newGroup.primitives.push_back(line);
                                do
                                {
                                    line = in.readLine();
                                    line.remove(QRegExp("[\t]*"));
                                }while(line[0] =='#');
                            }
                            do
                            {
                                line = in.readLine();
                                line.remove(QRegExp("[\t]*"));
                            }while(line[0] =='#');
                        }
                        newTask.groups.push_back(newGroup);
                    }
                    taskList.push_back(newTask);
                }
            }
        }
    }
}

void motion_selector::populateTree()
{
    for(int tsk = 0; tsk < taskList.size(); tsk++)
    {
        QTreeWidgetItem* newTaskItem = new QTreeWidgetItem();
        newTaskItem->setText(0,taskList[tsk].name);
        for(int grp = 0; grp < taskList[tsk].groups.size() ; grp++)
        {
            QTreeWidgetItem* newGroupItem = new QTreeWidgetItem();
            newGroupItem->setText(0,taskList[tsk].groups[grp].name);
            for(int prm = 0; prm < taskList[tsk].groups[grp].primitives.size(); prm++)
            {
                QTreeWidgetItem* newPrimitiveItem = new QTreeWidgetItem();
                newPrimitiveItem->setText(0,taskList[tsk].groups[grp].primitives[prm]);
                newGroupItem->addChild(newPrimitiveItem);
            }
            newTaskItem->addChild(newGroupItem);
        }
        treeItems.push_back(newTaskItem);
        ui->treeWidget->addTopLevelItem(newTaskItem);
    }
}

void motion_selector::on_sendCommand_clicked()
{
    if(ui->treeWidget->selectedItems().size() >0 && ui->treeWidget->selectedItems()[0]->childCount() == 0)
    {
        if(sliderVal != 0)
        {
            std::cout << "Currently selected = " << ui->treeWidget->selectedItems()[0]->text(0).toStdString() << std::endl;
            flor_control_msgs::FlorExecuteMotionRequest msg;
            msg.motion_name = (ui->treeWidget->selectedItems()[0])->text(0).toStdString();
            msg.time_factor = sliderVal;
            message_pub_.publish(msg);
        }
        else
            std::cout << "Invalid time factor." << std::endl;
    }
    else
        std::cout << "Please select something to send" << std::endl;
}

void motion_selector::printTasks()
{
    std::cout<< "printing task list..." <<std::endl;
    for(int tsk = 0;tsk < taskList.size();tsk++)
    {
        std::cout << taskList[tsk].name.toStdString() << std::endl;
        for(int grp = 0; grp < taskList[tsk].groups.size(); grp++)
        {
            std::cout << "  "<<taskList[tsk].groups[grp].name.toStdString() << std::endl;
            for(int prm = 0; prm < taskList[tsk].groups[grp].primitives.size(); prm++)
                std::cout << "      " << taskList[tsk].groups[grp].primitives[prm].toStdString() << std::endl;
        }
    }
}

motion_selector::~motion_selector()
{
    delete ui;
}
void motion_selector::on_timeFactorSlider_valueChanged(int value)
{
    sliderVal = (float)value/100.0;
    //std::cout << "slider value changed. now " << sliderVal << std::endl;
    ui->timeFactorLabel->setText(QString::number(sliderVal,'g',3));
}
