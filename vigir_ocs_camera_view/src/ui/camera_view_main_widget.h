#ifndef CAMERA_VIEW_MAIN_WIDGET_H
#define CAMERA_VIEW_MAIN_WIDGET_H

#include <QWidget>
#include <QPushButton>

#include <map>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

namespace Ui
{
    class CameraViewMainWidget;
}

class CameraViewMainWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit CameraViewMainWidget(QWidget *parent = 0);
    ~CameraViewMainWidget();

    virtual bool eventFilter( QObject * o, QEvent * e );

public Q_SLOTS:
    void oneViewToggle();
    void fourViewToggle();
    void updatePitch(int value);
    
private:
    Ui::CameraViewMainWidget *ui;

    std::map<std::string,QWidget*> views_list;

    QWidget* position_widget_;
    QPushButton* four_view_button_;
    QPushButton* one_view_button_;

    QString icon_path_;

};

#endif // CAMERA_VIEW_MAIN_WIDGET_H
