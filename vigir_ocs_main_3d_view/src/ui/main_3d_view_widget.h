#ifndef MAIN_3D_VIEW_WIDGET_H
#define MAIN_3D_VIEW_WIDGET_H

#include <QWidget>

namespace Ui
{
    class Main3DViewWidget;
}

class Main3DViewWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit Main3DViewWidget(QWidget *parent = 0);
    ~Main3DViewWidget();
    
private:
    Ui::Main3DViewWidget *ui;
};

#endif // MAIN_3D_VIEW_WIDGET_H
