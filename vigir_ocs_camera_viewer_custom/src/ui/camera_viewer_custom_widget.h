#ifndef CameraViewerCustomWidget_H
#define CameraViewerCustomWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>

#include <QPainter>


namespace Ui {
class CameraViewerCustomWidget;
}

class CameraViewerCustomWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit CameraViewerCustomWidget(QWidget *parent = 0);
    ~CameraViewerCustomWidget();
    
private:
    Ui::CameraViewerCustomWidget*ui;

Q_SIGNALS:
    void pitchChanged(int);

public Q_SLOTS:
    void alterDisplay(int);
    void updatePitch(int);
    void updateFeedFPS(int);
    void updateSelectedFPS(int);
    void scan();
    void isLocked();
    void alterChoices(int);


};

#endif // CameraViewerCustomWidget_H
