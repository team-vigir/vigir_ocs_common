#ifndef VIDEO_RECORD_WIDGET_H
#define VIDEO_RECORD_WIDGET_H

#include <QWidget>
#include "ros/ros.h"

namespace Ui {
class video_record_widget;
}

class video_record_widget : public QWidget
{
    Q_OBJECT

public:
    explicit video_record_widget(QWidget *parent = 0);
    ~video_record_widget();

public Q_SLOTS:
    void on_saveButton_clicked();
    void on_recordButton_clicked();

private:
    Ui::video_record_widget *ui;
    void createExperimentFile();
    void startFfmpegRecordingScript(int cameraNum);
    void getRobotLogs(double duration, std::string location);
    void startRawRecordingScript(int cameraNum);


    pid_t recordCam1;
    pid_t recordCam2;
    pid_t recordCam3;
    pid_t recordCam4;

    time_t endTime;
    time_t startTime;
};

#endif // VIDEO_RECORD_WIDGET_H
