#ifndef CAMERA_VIEW_WIDGET_H
#define CAMERA_VIEW_WIDGET_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>

#include <QPainter>


namespace Ui {
class Camera_View_Widget;
}

class Camera_View_Widget : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit Camera_View_Widget(QWidget *parent = 0);
    ~Camera_View_Widget();
    
private:
    Ui::Camera_View_Widget*ui;
    QRadioButton *button;
    QRadioButton *dynamicButton;
    QRadioButton *staticButton;
    QSpinBox * frameBox;
    QSpinBox * timeBox;

    QComboBox *camera;
  //  QWidget* picture;

//    QPoint initialPoint;
//    QPoint finalPoint;
  //  bool mouseClicked;

public slots:
    void enableSpin();
    void disableSpin();
    void alterDisplay(int);
   // void changeValue();

protected:
//    void QWidget::mouseReleaseEvent(QMouseEvent *mouse);
//    void QWidget::mousePressEvent(QMouseEvent *mouse);

};

#endif // CAMERA_VIEW_WIDGET_H
