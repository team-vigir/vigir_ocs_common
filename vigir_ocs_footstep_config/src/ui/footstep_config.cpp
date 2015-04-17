#include "footstep_config.h"
#include "ui_footstep_config.h"


FootstepConfigure::FootstepConfigure(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FootstepConfigure)
{
    ui->setupUi(this);   
    //connect to update all footstep paramaters when any part of ui is updated.
    connect(ui->maxTime,SIGNAL(valueChanged(double)),this,SLOT(updateFootstepParamaters(double)));
    connect(ui->maxSteps,SIGNAL(valueChanged(double)),this,SLOT(updateFootstepParamaters(double)));
    connect(ui->pathLengthRatio,SIGNAL(valueChanged(double)),this,SLOT(updateFootstepParamaters(double)));
    connect(ui->editStepMode,SIGNAL(currentIndexChanged(int)),this,SLOT(updateFootstepParamaters(int)));

    std::string ip = ros::package::getPath("vigir_ocs_main_view")+"/icons/";
    icon_path_ = QString(ip.c_str());

    //set down arrow for interaction box
    QString stylesheet = ui->editStepMode->styleSheet() + "\n" +
                "QComboBox::down-arrow {\n" +
                " image: url(" + icon_path_ + "down_arrow.png" + ");\n" +
                "}";
    ui->editStepMode->setStyleSheet(stylesheet);

//    Q_EMIT sendFootstepParamaters(getMaxTime(),getMaxSteps(),getPlanLengthRatio(),getFootstepInteraction(),patternGenerationEnabled());
//    ROS_ERROR("SEND time: %f steps: %d ratio:%f intmode: %d pattern %d",getMaxTime(),getMaxSteps(),getPlanLengthRatio(),getFootstepInteraction(),patternGenerationEnabled());
}

FootstepConfigure::~FootstepConfigure()
{
    delete ui;
}

//need slots to comply with qt signals...
void FootstepConfigure::updateFootstepParamaters(int ignore)
{
    Q_EMIT sendFootstepParamaters(ui->maxTime->value(),(int)ui->maxSteps->value(),ui->pathLengthRatio->value(),ui->editStepMode->currentIndex());
}

void FootstepConfigure::updateFootstepParamaters(double ignore)
{
    Q_EMIT sendFootstepParamaters(ui->maxTime->value(),(int)ui->maxSteps->value(),ui->pathLengthRatio->value(),ui->editStepMode->currentIndex());
}
