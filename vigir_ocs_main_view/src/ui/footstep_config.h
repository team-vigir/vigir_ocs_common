#ifndef FOOTSTEO_CONFIGURE_H
#define FOOTSTEO_CONFIGURE_H

#include <QWidget>
#include <ros/ros.h>


namespace Ui
{
    class FootstepConfigure;
}

class FootstepConfigure : public QWidget
{
    Q_OBJECT
    
public:
    explicit FootstepConfigure(QWidget *parent = 0);
    virtual ~FootstepConfigure();

    double getMaxTime();
    int getMaxSteps();
    double getPlanLengthRatio();
    int getFootstepInteraction();
    int getFootstepPlanningParameter();
    bool patternGenerationEnabled();

public Q_SLOTS:
    void updateFootstepParamaters();
    void updateFootstepParamaters(int ignore);
    void updateFootstepParamaters(double ignore);

Q_SIGNALS:
    void sendFootstepParamters(double,int,double,int,int,bool);

private:
    Ui::FootstepConfigure *ui;


};

#endif //FOOTSTEO_CONFIGURE_H
