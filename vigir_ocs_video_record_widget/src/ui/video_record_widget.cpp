#include "video_record_widget.h"
#include "ui_video_record_widget.h"
#include "boost/filesystem.hpp"
#include "boost/lexical_cast.hpp"
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <ctime>
#include "QMessageBox"
#include "fstream"

video_record_widget::video_record_widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::video_record_widget)
{
    ui->setupUi(this);
    ui->recordButton->setEnabled(false);

    ros::NodeHandle nh_("~");
    nh_.param<int>("video_record_widget/robot/sendPort",robotSendPort,3025);
    nh_.param<int>("video_record_widget/robot/recievePort",robotRecievePort,3024);
    nh_.param<std::string>("video_record_widget/robot/address",robotAddress,"10.66.171.30");
    //the_atlas = new AtlasInterface;
}

video_record_widget::~video_record_widget()
{
    delete ui;
}

void video_record_widget::on_saveButton_clicked()
{
    if(ui->saveButton->text() == "Save")
    {
        boost::filesystem3::path folder (std::string("/home/vigir/Experiments/"+ui->experimentName->text().toStdString()));
        if(boost::filesystem3::exists(folder))
        {
            std::cout << "Folder already exsists " << folder.c_str() << std::endl;
            QMessageBox msg;
            msg.setWindowTitle(QString::fromStdString("Video Recorder Error"));
            msg.setInformativeText(QString::fromStdString("Cannot have two experiments with the same name.\n\nPlease rename the experiment to continue."));
            msg.exec();
        }
        else
        {
            if(boost::filesystem3::create_directory(folder))
                std::cout<< "Created new folder at " << folder.c_str() << std::endl;
            ui->recordButton->setEnabled(true);
            ui->recordButton->setStyleSheet("color: rgb(0,255,0)");
	    ui->experimentName->setEnabled(false);
        }
    }
    else
    {
        std::string dir = "/home/vigir/Experiments/"+ui->experimentName->text().toStdString();
        std::string name = dir+"/experiment.txt";
        std::string summary = dir+"/summary.txt";
        FILE * summaryFile;
        summaryFile = fopen(summary.c_str(),"w+");
        if(summaryFile == NULL)
        {
            std::cout << "Failed to create Summary File." << std::endl;
            return;
        }
        else
        {
            std::ifstream experimentFile(name.c_str());
            std::string line;
            for(int i=0;i<4;i++)
            {
                std::getline(experimentFile,line);
                fprintf(summaryFile,"%s\n",line.c_str());
            }
            struct tm * timeinfo;
            time(&endTime);
            timeinfo = localtime( &endTime);
            fprintf(summaryFile," <EndTime>%d-%d-%d_%d:%d:%d</EndTime>\n",timeinfo->tm_mon,timeinfo->tm_mday,timeinfo->tm_year+1900,timeinfo->tm_hour,timeinfo->tm_min,timeinfo->tm_sec);
            std::cout << "Length of time recorded was " << difftime(endTime,startTime) << " seconds"<< std::endl;
            while(std::getline(experimentFile,line))
            {
                fprintf(summaryFile,"%s\n",line.c_str());
                if(line == " </Cameras>")
                    break;
            }
            fprintf(summaryFile," <Results>%s</Results>/n",ui->results->document()->toPlainText().toStdString().c_str());
            fprintf(summaryFile,"</Experiment>");
            ui->camera1Box->setEnabled(true);
            ui->camera2Box->setEnabled(true);
            ui->camera3Box->setEnabled(true);
            ui->camera4Box->setEnabled(true);
            ui->saveButton->setText(QString::fromStdString("Save"));
	    ui->experimentName->setEnabled(true);

            QMessageBox::StandardButton getLogs;
            getLogs = QMessageBox::question(this,"Video Record Widget", "Download Robot Logs?",QMessageBox::Yes|QMessageBox::No);
            if(getLogs == QMessageBox::Yes)
                getRobotLogs(difftime(endTime,startTime), dir);
        }
    }
}
void video_record_widget::getRobotLogs(double durration, std::string location)
{
    std::cout << "Getting robot logs...." << std::endl;
    /*AtlasErrorCode robotError;
    robotError =  the_atlas->open_net_connection_to_robot(robotAddress,robotSendPort,robotRecievePort);
    if(robotError == AtlasRobot ::NO_ERRORS)
    {
        robotError = the_atlas->download_robot_log_file(location,durration);
        if(robotError != AtlasRobot::NO_ERRORS)
            displayRobotError(robotError);
        else
            std::cout << "Robot Logs saved successfully." << std::endl;
    }
    else
        displayRobotError(robotError);
*/
}

/*void video_record_widget::displayRobotError(AtlasErrorCode err)
{
    QMessageBox msg;
    msg.setWindowTitle(QString::fromStdString("Video Recorder Error"));
    msg.setInformativeText(QString::fromStdString(the_atlas->get_error_code_text(err)));
}*/

void video_record_widget::on_recordButton_clicked()
{
    if(ui->recordButton->text() == "Record")
    {
        ui->saveButton->setEnabled(false);
        ui->recordButton->setStyleSheet("color: rgb(255,0,0)");
        ui->recordButton->setText("Stop Recording");
        createExperimentFile();
        ui->camera1Box->setEnabled(false);
        ui->camera2Box->setEnabled(false);
        ui->camera3Box->setEnabled(false);
        ui->camera4Box->setEnabled(false);
        if(ui->camera1Box->isChecked())
            startRawRecordingScript(1);
        if(ui->camera2Box->isChecked())
            startRawRecordingScript(2);
        if(ui->camera3Box->isChecked())
            startRawRecordingScript(3);
        if(ui->camera4Box->isChecked())
            startRawRecordingScript(4);
    }
    else
    {
        std::cout << "Killing camera recording processes...." << std::endl;
        if(ui->camera1Box->isChecked())
            kill(-recordCam1,SIGTERM);
        if(ui->camera2Box->isChecked())
            kill(-recordCam2,SIGTERM);
        if(ui->camera3Box->isChecked())
            kill(-recordCam3,SIGTERM);
        if(ui->camera4Box->isChecked())
            kill(-recordCam4,SIGTERM);
        ui->saveButton->setEnabled(true);
        ui->saveButton->setText(QString::fromStdString("Save Summary"));
        ui->recordButton->setText(QString::fromStdString("Record"));
        ui->recordButton->setStyleSheet("color: rgb(0,0,0)");
        ui->recordButton->setEnabled(false);
    }
}

void video_record_widget::createExperimentFile()
{
    std::cout << "Creating experiment.txt file....." << std::endl;
    FILE * experimentTxt;
    std::string name = "/home/vigir/Experiments/"+ui->experimentName->text().toStdString()+"/experiment.txt";
    experimentTxt = fopen(name.c_str(),"w+");
    if(experimentTxt == NULL)
    {
        std::cout << "Failed to create experiments.txt file." << std::endl;
        return;
    }
    fprintf(experimentTxt,"<?xml version=\"1.0\"?>\n");
    fprintf(experimentTxt,"<Experiment>\n <Name>%s</Name>\n",ui->experimentName->text().toStdString().c_str());

    struct tm * timeinfo;
    time(&startTime);
    timeinfo = localtime( &startTime);
    fprintf(experimentTxt," <StartTime>%d-%d-%d_%d:%d:%d</StartTime>\n",timeinfo->tm_mon,timeinfo->tm_mday,timeinfo->tm_year+1900,timeinfo->tm_hour,timeinfo->tm_min,timeinfo->tm_sec);

    fprintf(experimentTxt," <Description>%s</Description>\n",ui->descriptionTxtBox->document()->toPlainText().toStdString().c_str());

    fprintf(experimentTxt," <Cameras>\n");
    if(ui->camera1Box->isChecked())
        fprintf(experimentTxt,"  <CameraName>Camera 1</CameraName>\n");
    if(ui->camera2Box->isChecked())
        fprintf(experimentTxt,"  <CameraName>Camera 2</CameraName>\n");
    if(ui->camera3Box->isChecked())
        fprintf(experimentTxt,"  <CameraName>Camera 3</CameraName>\n");
    if(ui->camera4Box->isChecked())
        fprintf(experimentTxt,"  <CameraName>Camera 4</CameraName>\n");
    fprintf(experimentTxt," </Cameras>\n");
    fprintf(experimentTxt,"</Experiment>");
    fclose(experimentTxt);
    std::cout << "Finished setting up Experiments.txt now starting camera scripts..." << std::endl;
}

void video_record_widget::startFfmpegRecordingScript(int cameraNum)
{
    std::cout << "Start ffmpeg record script called on camera " << cameraNum <<std::endl;
    std::ifstream scriptTest;
    std::string scriptLoc = "/opt/vigir/catkin_ws/src/vigir_ocs_common/vigir_ocs_video_record_widget/src/scripts/camera"+boost::lexical_cast<std::string>(cameraNum)+"Ffmpeg.sh";
    std::string experimentDirectory = "/home/vigir/Experiments/"+ui->experimentName->text().toStdString();
    scriptTest.open(scriptLoc.c_str());
    if(!scriptTest.is_open())
    {
        std::cout << "Could not find ffmpeg recording script for the camera " << cameraNum << " at " << scriptLoc << std::endl;
        return;
    }
    scriptTest.close();
    std::cout<< "Start script found." << std::endl;
    pid_t temp;
    temp = fork();
    if(temp >=0)
    {
        if( temp == 0)
        {
            temp = setsid();
            std::string scriptCall = scriptLoc+" /home/vigir/Experiments/"+ui->experimentName->text().toStdString()+" camera"+boost::lexical_cast<std::string>(cameraNum);
            std::cout << scriptCall << std::endl;
            system(scriptCall.c_str());
        }
        else
            std::cout<< "Started recording script for Camera " << cameraNum;
        switch(cameraNum)
        {
        case 1:
            recordCam1 = temp;
            break;
        case 2:
            recordCam2 = temp;
            break;
        case 3:
            recordCam3 = temp;
            break;
        case 4:
            recordCam4 = temp;
        default:
            break;
        }
    }
    else
        std::cout << "Failed to make child process to start script in.." <<std::endl;
}

void video_record_widget::startRawRecordingScript(int cameraNum)
{
    std::cout << "Start Raw record script called on camera " << cameraNum <<std::endl;
    std::ifstream scriptTest;
    std::string scriptLoc = "/opt/vigir/catkin_ws/src/vigir_ocs_common/vigir_ocs_video_record_widget/src/scripts/camera"+boost::lexical_cast<std::string>(cameraNum)+"Raw.sh";
    std::string experimentDirectory = "/home/vigir/Experiments/"+ui->experimentName->text().toStdString();
    scriptTest.open(scriptLoc.c_str());
    if(!scriptTest.is_open())
    {
        std::cout << "Could not find Raw recording script for the camera " << cameraNum << " at " << scriptLoc << std::endl;
        return;
    }
    scriptTest.close();
    std::cout<< "Start script found." << std::endl;
    pid_t temp;
    temp = fork();
    if(temp >=0)
    {
        if( temp == 0)
        {
            temp = setsid();
            std::string scriptCall = scriptLoc+" /home/vigir/Experiments/"+ui->experimentName->text().toStdString()+" camera"+boost::lexical_cast<std::string>(cameraNum);
            std::cout << scriptCall << std::endl;
            system(scriptCall.c_str());
        }
        else
            std::cout<< "Started recording script for Camera " << cameraNum;
        switch(cameraNum)
        {
        case 1:
            recordCam1 = temp;
            break;
        case 2:
            recordCam2 = temp;
            break;
        case 3:
            recordCam3 = temp;
            break;
        case 4:
            recordCam4 = temp;
        default:
            break;
        }
    }
    else
        std::cout << "Failed to make child process to start script in.." <<std::endl;
}
