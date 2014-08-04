#ifndef MAP_REGION_CONFIGURE_H
#define MAP_REGION_CONFIGURE_H

#include <QWidget>
#include <ros/ros.h>


namespace Ui
{
    class MapRegionConfigure;
}

class MapRegionConfigure : public QWidget
{
    Q_OBJECT
    
public:
    explicit MapRegionConfigure(QWidget *parent = 0);
    virtual ~MapRegionConfigure();

    double getMinHeight();
    double getMaxHeight();
    double getResolution();

private:
    Ui::MapRegionConfigure *ui;

//public Q_SLOTS:



};

#endif // MAP_REGION_CONFIGURE_H
