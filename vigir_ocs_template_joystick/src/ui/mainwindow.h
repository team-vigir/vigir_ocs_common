#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QWidget>
#include <QtGui>
#include <QComboBox>
#include "Controller.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
protected:
    void keyPressEvent(QKeyEvent *);
    void keyReleaseEvent(QKeyEvent *);
private:
    Ui::MainWindow *ui;
    //handle multiple keys at once
    QSet<Qt::Key> keysPressed;
    void processKeys();
    vigir_ocs::Controller * controller;
    QSignalMapper * mapper;

public Q_SLOTS:
    void setDirection(QString s);
    void disableLeftLabel();
    void disableRightLabel();
    void selectTemplate();
    void populateTemplateComboBox(int tempId);
};

#endif // MAINWINDOW_H
