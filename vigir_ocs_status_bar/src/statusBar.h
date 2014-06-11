#ifndef STATUSBAR_H
#define STATUSBAR_H

#include <QMainWindow>
#include <QSpacerItem>
#include "glancehubSbar.h"
#include "logSbar.h"

namespace Ui {
class StatusBar;
}


class StatusBar : public QMainWindow
{
    Q_OBJECT

public:
    ~StatusBar();
    explicit StatusBar(QWidget *parent = 0);

private:
    Ui::StatusBar *ui;
    glancehubSbar * glanceSbar;
    LogSbar * logSbar;
   
public Q_SLOTS:
    void receivePositionText(QString s);

};

#endif // STATUSBAR_H
