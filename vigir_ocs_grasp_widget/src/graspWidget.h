#ifndef GRASPWIDGET_H
#define GRASPWIDGET_H

#include <QWidget>

namespace Ui {
class graspWidget;
}

class graspWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit graspWidget(QWidget *parent = 0);
    ~graspWidget();
    
private:
    Ui::graspWidget *ui;
};

#endif // GRASPWIDGET_H
