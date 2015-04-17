#ifndef FOOTSTEO_CONFIGURE_H
#define FOOTSTEO_CONFIGURE_H

#include <QWidget>
#include <ros/ros.h>
#include <ros/package.h>


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

public Q_SLOTS:
    void updateFootstepParamaters(int ignore);
    void updateFootstepParamaters(double ignore);

Q_SIGNALS:
    void sendFootstepParamaters(double,int,double,int);

private:
    Ui::FootstepConfigure *ui;
    QString icon_path_;


};

#endif //FOOTSTEO_CONFIGURE_H
