#ifndef STATUS_WINDOW_H
#define STATUS_WINDOW_H

#include<QPainter>
#include <QPalette>
#include <QWidget>
#include <jointList.h>
#include <robotStatus.h>
//#include <flor_ocs_msgs/2DPoint.h>
//#include <flor_ocs_msgs/CenterGrav.h>

namespace Ui {
class status_window;
}

class status_window : public QWidget
{
    Q_OBJECT
    
public:
    explicit status_window(QWidget *parent = 0);
    ~status_window();
    
private Q_SLOTS:
    void on_showJointButton_clicked();
    void on_showRobotStatus_clicked();

protected:
    void paintEvent(QPaintEvent *);
private:
    Ui::status_window *ui;
    QPoint points[];
    QPoint cg;
    int numberOfPoints;
    QPalette palette;
    QPainter painter;
    robotStatus* rbtStatus;
    jointList* jntList;
    QWidget* par;
};

#endif // STATUS_WINDOW_H
