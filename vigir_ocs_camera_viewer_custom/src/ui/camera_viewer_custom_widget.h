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

    virtual bool eventFilter( QObject * o, QEvent * e );
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

    void setFeedToSingleImage();
    void setAreaToSingleImage();

    void disableImagePanel(bool);
    void enableDisplayGroup(bool);
    void disableCameraPanel(bool);
    void disableHeadPanel(bool);
    void disableAreaFeedPanel(bool);
    void disableFeedPanel(bool);
/**
protected:
    void mouseMoveEvent(QMouseEvent*);**/


};

#endif // CameraViewerCustomWidget_H
