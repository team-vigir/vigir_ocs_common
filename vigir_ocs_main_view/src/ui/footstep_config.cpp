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


