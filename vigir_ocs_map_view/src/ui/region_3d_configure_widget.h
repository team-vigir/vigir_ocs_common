#ifndef REGION_3D_CONFIGURE_H
#define REGION_3D_CONFIGURE_H

#include <QWidget>
#include <ros/ros.h>

namespace Ui
{
    class Region3DConfigure;
}

class Region3DConfigure : public QWidget
{
    Q_OBJECT
    
public:
    explicit Region3DConfigure(QWidget *parent = 0);
    virtual ~Region3DConfigure();
    double getMinHeight();
    double getMaxHeight();
    double getVoxelResolution();
    int getAggregSize();


private:
    Ui::Region3DConfigure *ui;

//public Q_SLOTS:



};

#endif // REGION_3D_CONFIGURE_H
