#ifndef HANDOFFSETWIDGET_H
#define HANDOFFSETWIDGET_H

#include <QWidget>

//#include <graspWidget.h>

namespace Ui {
class handOffsetWidget;
}

class handOffsetWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit handOffsetWidget(QWidget *parent = 0);
    ~handOffsetWidget();
    
public Q_SLOTS:
    void on_roll_inc_clicked();
    void on_roll_dec_clicked();

private:
    Ui::handOffsetWidget *ui;

    int roll;
};

#endif // HANDOFFSETWIDGET_H
