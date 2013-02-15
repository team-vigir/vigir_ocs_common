#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>

#include <QPainter>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow();
    
private:
    Ui::MainWindow *ui;
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

#endif // MAINWINDOW_H
