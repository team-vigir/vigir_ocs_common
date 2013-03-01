#ifndef STATUS_WINDOW_H
#define STATUS_WINDOW_H

#include<QPainter>
#include <QPalette>
#include <QWidget>
#include <jointList.h>

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

private:
    Ui::status_window *ui;
    QPalette palette;
    QPainter painter;
    jointList* jntList;
    QWidget* par;
    void paint();
};

#endif // STATUS_WINDOW_H
