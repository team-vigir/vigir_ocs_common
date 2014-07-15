#include "manipulationTest.h"
#include <unistd.h>


//bug: templates will not be deleted correctly if the main view is closed and restarted during the test. A full close of ocs is fine

ManipulationTest::ManipulationTest(QWidget *parent) :
    MainViewWidget(parent)
{    
    // create a publisher to add templates
    template_add_pub_   = n_.advertise<flor_ocs_msgs::OCSTemplateAdd>( "/template/add", 5, false );
    startingPosition = new QVector3D(1,1,1);
    startingRotation = new QQuaternion();

    elapsedTimer = new QElapsedTimer();
    //accounts for current challenge in vector
    challengeCount = 0;
    defineTransforms();

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d_%m_%Y__%I_%M_%S",timeinfo);
    //std::string str(buffer);
    QString str(buffer);  

    //make file with current time as name
    results = new QFile(QDir::homePath()+"/ManipulationResults_" + str);    
    if(!results->open(QIODevice::WriteOnly | QIODevice::Text))
        ROS_ERROR("Could not open file: %s",qPrintable(QString(QDir::homePath()+"/ManipulationResults_" + str)));

    template_list_sub = n_.subscribe<flor_ocs_msgs::OCSTemplateList>( "/template/list", 5, &ManipulationTest::templateListCb ,this );
    template_remove_pub_ = n_.advertise<flor_ocs_msgs::OCSTemplateRemove>( "/template/remove", 5, false );

    interactive_marker_sub = n_.subscribe<flor_ocs_msgs::OCSInteractiveMarkerAdd>("/flor/ocs/interactive_marker_server/add",5,&ManipulationTest::addInteractiveMarker,this);

    startChallenge = new QPushButton("Start New Challenge",this);
    this->layout()->addWidget(startChallenge);
    connect(startChallenge,SIGNAL(pressed()),this,SLOT(newChallenge()));

    template_update_pub = n_.advertise<flor_ocs_msgs::OCSTemplateUpdate>( "/template/update", 5, false );

}

void ManipulationTest::addInteractiveMarker( const flor_ocs_msgs::OCSInteractiveMarkerAdd::ConstPtr& msg )
{
    static int markerCounter = 0;
    if(msg->topic.find("/template_pose_") != std::string::npos)
    {
        int size = msg->topic.size()-15;
        QString num( msg->topic.substr(15,size).c_str());
        if((num.toInt() + 1) % 2 != 0)//odd?
        {
            insertTemplate(templatePositions[challengeCount], templateRotations[challengeCount],templateNames[challengeCount]);
            //ROS_ERROR("template %s",qPrintable(templateNames[challengeCount]));
            challengeCount++;
        }
    }
}

void ManipulationTest::templateListCb(const flor_ocs_msgs::OCSTemplateList::ConstPtr& msg)
{    
    temList = *msg;    
}

//initializing 10 predefined transforms for templates to be placed. also ten different templates by name
void ManipulationTest::defineTransforms()
{
    //make 40 tests
//    for(int i =0;i<4;i++)
//    {
        //stash all into corresponding vector
        QVector3D * position1 = new QVector3D(5,3,1);
        QQuaternion * rotation1 = new QQuaternion(.5,.4,.7,.2);
        templatePositions.push_back(position1);
        templateRotations.push_back(rotation1);
        templateNames.push_back("debris/2x4x36.mesh");

        QVector3D * position2 = new QVector3D(-2,3,5);
        QQuaternion * rotation2 = new QQuaternion(.2,.5,.3,.1);
        templatePositions.push_back(position2);
        templateRotations.push_back(rotation2);
        templateNames.push_back("door/door.mesh");

        QVector3D * position3 = new QVector3D(4,-1,2);
        QQuaternion * rotation3 = new QQuaternion(.2,.8,.3,.7);
        templatePositions.push_back(position3);
        templateRotations.push_back(rotation3);
        templateNames.push_back("valve/valve.mesh");

        QVector3D * position4 = new QVector3D(3,3,1);
        QQuaternion * rotation4 = new QQuaternion(.5,.4,.4,.2);
        templatePositions.push_back(position4);
        templateRotations.push_back(rotation4);
        templateNames.push_back("debris/4x4x24.mesh");

        QVector3D * position5 = new QVector3D(-3,4,6);
        QQuaternion * rotation5 = new QQuaternion(.5,.4,.7,.2);
        templatePositions.push_back(position5);
        templateRotations.push_back(rotation5);
        templateNames.push_back("tools/DRC_drill.mesh");

        QVector3D * position6 = new QVector3D(4,-2,2);
        QQuaternion * rotation6 = new QQuaternion(.2,.4,.1,.2);
        templatePositions.push_back(position6);
        templateRotations.push_back(rotation6);
        templateNames.push_back("valve/QUAL_valve_holes.mesh");

        QVector3D * position7 = new QVector3D(5,3,2);
        QQuaternion * rotation7 = new QQuaternion(.5,.4,.2,.2);
        templatePositions.push_back(position7);
        templateRotations.push_back(rotation7);
        templateNames.push_back("tools/cordless_drill.mesh");

        QVector3D * position8 = new QVector3D(-3,1,2);
        QQuaternion * rotation8 = new QQuaternion(.1,.4,.7,.2);
        templatePositions.push_back(position8);
        templateRotations.push_back(rotation8);
        templateNames.push_back("door/doorhandle.mesh");

        QVector3D * position9 = new QVector3D(-5,4,3);
        QQuaternion * rotation9 = new QQuaternion(.5,.1,.1,.2);
        templatePositions.push_back(position9);
        templateRotations.push_back(rotation9);
        templateNames.push_back("tools/ladder.mesh");

        QVector3D * position10 = new QVector3D(1,-1,2);
        QQuaternion * rotation10 = new QQuaternion(.5,.2,.3,.1);
        templatePositions.push_back(position10);
        templateRotations.push_back(rotation10);
        templateNames.push_back("hose/coupling_hexagon.mesh");
    //}
}



void ManipulationTest::newChallenge()
{    
    if(challengeCount < templatePositions.size())
    {
        //write info about template locations, retemplate_id_liststart timer0,0,0,0
        qint64 challengeTime = elapsedTimer->restart();

        if(temList.template_id_list.size() != 0)
        {
            static int templateListCounter = 0;
            QString timeStr = QString::number(challengeTime);
            //grab string form of goal position
            QString goalPosXStr = QString::number(temList.pose[1].pose.position.x );
            QString goalPosYStr = QString::number(temList.pose[1].pose.position.y );
            QString goalPosZStr = QString::number(temList.pose[1].pose.position.z );
            QString goalRotXStr = QString::number(temList.pose[1].pose.orientation.x );
            QString goalRotYStr = QString::number(temList.pose[1].pose.orientation.y );
            QString goalRotZStr = QString::number(temList.pose[1].pose.orientation.z );
            QString goalRotWStr = QString::number(temList.pose[1].pose.orientation.w );
            templateListCounter++;
            //grab strings of actual position
            QString actualPosXStr = QString::number(temList.pose[0].pose.position.x );
            QString actualPosYStr = QString::number(temList.pose[0].pose.position.y );
            QString actualPosZStr = QString::number(temList.pose[0].pose.position.z );
            QString actualRotXStr = QString::number(temList.pose[0].pose.orientation.x );
            QString actualRotYStr = QString::number(temList.pose[0].pose.orientation.y );
            QString actualRotZStr = QString::number(temList.pose[0].pose.orientation.z );
            QString actualRotWStr = QString::number(temList.pose[0].pose.orientation.w );
            templateListCounter++;

            QString testResults ="Challenge :,"+ QString::number(challengeCount) +  ", time: ," + timeStr  +",Position Goal: ,"+ goalPosXStr + ", " +
                    goalPosYStr + "," + goalPosZStr + ", Rotation Actual: " + goalRotXStr + ", " +
                    goalRotYStr + "," + goalRotZStr + ", " + goalRotWStr + ", Actual Position: ," +  actualPosXStr + ", " +
                    actualPosYStr + ", " + actualPosZStr + ", Actual Rotation: ," + actualRotXStr + ", " + actualRotYStr + ", " + actualRotZStr + ", " + actualRotWStr +"\n";
            results->write(testResults.toStdString().c_str());

        }


        //remove templates if present
        if(challengeCount > 0)
        {            
            //cant reliably remove templates so move them farrrrrr awwwayy
            flor_ocs_msgs::OCSTemplateUpdate msg;
            msg.template_id = temList.template_id_list[challengeCount*2-2];

            //copy rotation first
            msg.pose.pose.orientation.x = 1;
            msg.pose.pose.orientation.y = 0;
            msg.pose.pose.orientation.z = 0;
            msg.pose.pose.orientation.w = 0;

            //update coordinates based on latest data from list
            msg.pose.pose.position.x = 10001;
            msg.pose.pose.position.y = 10001;
            msg.pose.pose.position.z = 10001;

            template_update_pub.publish(msg);

            usleep(1000000);
            flor_ocs_msgs::OCSTemplateUpdate msg2;
            msg2.template_id = temList.template_id_list[challengeCount*2-1];

            //copy rotation first
            msg2.pose.pose.orientation.x = 1;
            msg2.pose.pose.orientation.y = 0;
            msg2.pose.pose.orientation.z = 0;
            msg2.pose.pose.orientation.w = 0;

            //update coordinates based on latest data from list
            msg2.pose.pose.position.x = 10001;
            msg2.pose.pose.position.y = 10001;
            msg2.pose.pose.position.z = 10001;

            template_update_pub.publish(msg2);

        }


        //insert two templates, one goal, one to be manipulated
        insertTemplate(startingPosition,startingRotation, templateNames[challengeCount]);
       // usleep(500000);
    }
    else
    {
        if(results->isOpen())
            results->close();

        //remove last templates
        flor_ocs_msgs::OCSTemplateUpdate msg;
        msg.template_id = temList.template_id_list[challengeCount*2-2];

        //copy rotation first
        msg.pose.pose.orientation.x = 1;
        msg.pose.pose.orientation.y = 0;
        msg.pose.pose.orientation.z = 0;
        msg.pose.pose.orientation.w = 0;

        //update coordinates based on latest data from list
        msg.pose.pose.position.x = 10001;
        msg.pose.pose.position.y = 10001;
        msg.pose.pose.position.z = 10001;

        template_update_pub.publish(msg);

        usleep(1000000);
        flor_ocs_msgs::OCSTemplateUpdate msg2;
        msg2.template_id = temList.template_id_list[challengeCount*2-1];

        //copy rotation first
        msg2.pose.pose.orientation.x = 1;
        msg2.pose.pose.orientation.y = 0;
        msg2.pose.pose.orientation.z = 0;
        msg2.pose.pose.orientation.w = 0;

        //update coordinates based on latest data from list
        msg2.pose.pose.position.x = 10001;
        msg2.pose.pose.position.y = 10001;
        msg2.pose.pose.position.z = 10001;

        template_update_pub.publish(msg2);

        startChallenge->setText("You're Done! File is written to home directory.");
        startChallenge->setEnabled(false);

    }
}

void ManipulationTest::insertTemplate(QVector3D * position, QQuaternion * rotation, QString path)
{
    flor_ocs_msgs::OCSTemplateAdd cmd;
    geometry_msgs::PoseStamped pose;

    cmd.template_path = path.toStdString();

    pose.pose.position.x = position->x();
    pose.pose.position.y = position->y();
    pose.pose.position.z = position->z();
    pose.pose.orientation.x = rotation->x();
    pose.pose.orientation.y = rotation->y();
    pose.pose.orientation.z = rotation->z();
    pose.pose.orientation.w = rotation->scalar();

    pose.header.frame_id = "/world";
    pose.header.stamp = ros::Time::now();

    cmd.pose = pose;

    // publish complete list of templates and poses
    template_add_pub_.publish( cmd );
}
ManipulationTest::~ManipulationTest()
{

}

