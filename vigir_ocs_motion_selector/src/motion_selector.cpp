#include "motion_selector.h"
#include "ui_motion_selector.h"
#include <flor_control_msgs/FlorExecuteMotionRequest.h>
#include <qfile.h>
#include <QTextStream>
#include <QTreeWidget>
#include <QRegExp>
#include <QStringList>
#include <QBoxLayout>
#include <ros/package.h>

motion_selector::motion_selector(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::motion_selector)
{
    ui->setupUi(this);
    sliderVal = 1.00;
    processTextFile(QString::fromStdString(ros::package::getPath("vigir_ocs_motion_selector") + "/src/tree.txt"));
    setupQuickButtons(QString::fromStdString(ros::package::getPath("vigir_ocs_motion_selector") + "/src/quickButtons.txt"));
    //ros::
    std::cout << "File done processing now populatin the tree." << std::endl;
    populateTree();

    message_pub_ = nh_.advertise<flor_control_msgs::FlorExecuteMotionRequest>( "/flor/motion_service/motion_command",1,false);

    //key_event_sub_ = nh_.subscribe<vigir_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &motion_selector::processNewKeyEvent, this );

    timer.start(33, this);
}

motion_selector::~motion_selector()
{
    delete ui;
}

void motion_selector::timerEvent(QTimerEvent *event)
{
	// check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();
    
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

void motion_selector::on_enableQuickButtons_clicked()
{
    for(int i=0;i<quickButtonList.size();i++)
    {
        quickButtonList[i]->button->setEnabled(ui->enableQuickButtons->isChecked());
        quickButtonList[i]->timeFactor->setEnabled(ui->enableQuickButtons->isChecked());
    }
}

void motion_selector::setupQuickButtons(QString path)
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
                QStringList strings;
                strings = line.split(',');
                if(strings.size() == 3)
                {
                    if(strings[0] == QString::fromStdString("space") || strings[0] == QString::fromStdString("Space"))
                    {
                        switch(strings[1].toInt())
                        {
                        case 1:
                            ui->qb1->insertSpacing(1,strings[2].toInt());
                            break;
                        case 2:
                            ui->qb2->insertSpacing(1,strings[2].toInt());
                            break;
                        case 3:
                            ui->qb3->insertSpacing(1,strings[2].toInt());
                            break;
                        default:
                            std::cout << "malformed spacing in config." << std::endl;
                        }
                    }
                    else
                    {
                        quickButton* newQB = new quickButton();
                        newQB->button = new QPushButton();
                        newQB->timeFactor = new QDoubleSpinBox();
                        newQB->button->setText(strings[1]);
                        newQB->timeFactor->setValue(strings[2].toDouble());
                        newQB->button->setEnabled(false);
                        newQB->timeFactor->setEnabled(false);
                        newQB->timeFactor->setMaximumWidth(70);
                        connect(newQB->button, SIGNAL(clicked()),this,SLOT(quickButtonClicked()));
                        QVBoxLayout* layout = new QVBoxLayout;
                        layout->setDirection(QBoxLayout::LeftToRight);
                        layout->addWidget(newQB->button);
                        layout->addWidget(newQB->timeFactor);
                        quickButtonList.push_back(newQB);
                        switch(strings[0].toInt())
                        {
                        case 1:

                            ui->qb1->insertLayout(1,layout,0);
                            break;
                        case 2:
                            ui->qb2->insertLayout(1,layout,0);
                            break;
                        case 3:
                            ui->qb3->insertLayout(1,layout,0);
                            break;
                        default:
                            std::cout << "unknown entry in config for colum" << std::endl;
                        }
                    }

                }
            }
        }
    }
}

void motion_selector::quickButtonClicked()
{
    if(ui->enableQuickButtons->isChecked())
    {
        QPushButton *button = (QPushButton *)sender();
        std::cout << "Pressed button named " << button->text().toStdString() <<std::endl;
        flor_control_msgs::FlorExecuteMotionRequest msg;
        msg.motion_name = (button->text().toStdString());
        for(int i =0; i < quickButtonList.size();i++)
        {
            if(quickButtonList[i]->button->text() == button->text())
            {
                msg.time_factor = quickButtonList[i]->timeFactor->value();
                message_pub_.publish(msg);
                break;
            }
        }
    }
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

void motion_selector::on_timeFactorSlider_valueChanged(int value)
{
    sliderVal = (float)value/100.0;
    //std::cout << "slider value changed. now " << sliderVal << std::endl;
    ui->timeFactorLabel->setText(QString::number(sliderVal,'g',3));
}

//void motion_selector::addHotKeys()
//{
//    HotkeyManager::Instance()->addHotkeyFunction("ctrl+m",boost::bind(&motion_selector::toggleVisibilityHotkey,this));
//}

//void motion_selector::toggleVisibilityHotkey()
//{
//    if(this->isVisible())
//    {
//        this->hide();
//    }
//    else
//    {
//        //this->move(QPoint(key_event->cursor_x+5, key_event->cursor_y+5));
//        this->show();
//    }
//}

