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
    connect(ui->footstepInteractionModeBox,SIGNAL(currentIndexChanged(int)),this,SLOT(updateFootstepParamaters(int)));
    connect(ui->footstepPlanningParameterBox,SIGNAL(currentIndexChanged(int)),this,SLOT(updateFootstepParamaters(int)));
    connect(ui->patternGenerationEnabled,SIGNAL(clicked()),this,SLOT(updateFootstepParamaters()));

    std::string ip = ros::package::getPath("vigir_ocs_main_view")+"/icons/";
    icon_path_ = QString(ip.c_str());

    //set down arrow for param box
    QString stylesheet = ui->footstepPlanningParameterBox->styleSheet() + "\n" +
            "QComboBox::down-arrow {\n" +
            " image: url(" + icon_path_ + "down_arrow.png" + ");\n" +
            "}";
    ui->footstepPlanningParameterBox->setStyleSheet(stylesheet);
    stylesheet = ui->footstepInteractionModeBox->styleSheet() + "\n" +
            "QComboBox::down-arrow {\n" +
            " image: url(" + icon_path_ + "down_arrow.png" + ");\n" +
            "}";
    ui->footstepInteractionModeBox->setStyleSheet(stylesheet);
}

double FootstepConfigure::getMaxTime()
{
    return ui->maxTime->value();
}

int FootstepConfigure::getMaxSteps()
{
    return (int)ui->maxSteps->value();
}

double FootstepConfigure::getPlanLengthRatio()
{
    return ui->pathLengthRatio->value();
}

int FootstepConfigure::getFootstepInteraction()
{
    return ui->footstepInteractionModeBox->currentIndex();
}

int FootstepConfigure::getFootstepPlanningParameter()
{
    return ui->footstepPlanningParameterBox->currentIndex();
}

bool FootstepConfigure::patternGenerationEnabled()
{
    return ui->patternGenerationEnabled->isChecked();
}

//need slots to comply with qt signals...
void FootstepConfigure::updateFootstepParamaters(double ignore)
{
    Q_EMIT sendFootstepParamters(getMaxTime(),getMaxSteps(),getPlanLengthRatio(),getFootstepInteraction(),getFootstepPlanningParameter(),patternGenerationEnabled());
}
void FootstepConfigure::updateFootstepParamaters(int ignore)
{
    Q_EMIT sendFootstepParamters(getMaxTime(),getMaxSteps(),getPlanLengthRatio(),getFootstepInteraction(),getFootstepPlanningParameter(),patternGenerationEnabled());
}
void FootstepConfigure::updateFootstepParamaters()
{
    Q_EMIT sendFootstepParamters(getMaxTime(),getMaxSteps(),getPlanLengthRatio(),getFootstepInteraction(),getFootstepPlanningParameter(),patternGenerationEnabled());
}

FootstepConfigure::~FootstepConfigure()
{
    delete ui;
}


