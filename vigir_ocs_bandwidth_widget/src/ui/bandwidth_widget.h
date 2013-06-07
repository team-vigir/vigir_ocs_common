#ifndef BandwidthWidget_H
#define BandwidthWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>
#include <QStringList>

#include <QPainter>
#include <QtGui>

#include <flor_ocs_msgs/OCSBandwidth.h>
#include <ros/ros.h>

namespace Ui {
class BandwidthWidget;
}

class BandwidthWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit BandwidthWidget(QWidget *parent = 0);
    ~BandwidthWidget();

    void processBandwidthMessage(const flor_ocs_msgs::OCSBandwidth::ConstPtr& msg);

private:
    Ui::BandwidthWidget* ui;

    ros::NodeHandle nh_;
    ros::Subscriber ocs_bandwidth_sub_;
    
    typedef struct
    {
    	std::string node_name;
    	unsigned long total_bytes_read;
    	unsigned long total_bytes_sent;
	} BandwidthStruct;
    
    std::vector<BandwidthStruct> node_bandwidth_info_;

protected:
    void timerEvent(QTimerEvent *event);
private:
    QBasicTimer timer;
};

#endif // BandwidthWidget_H
