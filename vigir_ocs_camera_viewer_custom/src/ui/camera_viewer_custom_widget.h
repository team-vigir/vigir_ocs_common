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
    /*QRadioButton *button;
    QRadioButton *dynamicButton;
    QRadioButton *staticButton;
    QSpinBox * frameBox;
    QSpinBox * timeBox;*/

    QComboBox *camera;
  //  QWidget* picture;

//    QPoint initialPoint;
//    QPoint finalPoint;
  //  bool mouseClicked;

/*public slots:
    void enableSpin();
    void disableSpin();*/
    void alterDisplay(int);
   // void changeValue();

protected:
//    void QWidget::mouseReleaseEvent(QMouseEvent *mouse);
//    void QWidget::mousePressEvent(QMouseEvent *mouse);

};

#endif // CameraViewerCustomWidget_H
