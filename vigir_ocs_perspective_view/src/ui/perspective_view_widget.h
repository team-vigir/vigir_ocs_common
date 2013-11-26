#ifndef PERSPECTIVE_VIEW_WIDGET_H
#define PERSPECTIVE_VIEW_WIDGET_H

#include <QWidget>

namespace Ui
{
    class PerspectiveViewWidget;
}

class PerspectiveViewWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit PerspectiveViewWidget(QWidget *parent = 0);
    ~PerspectiveViewWidget();

    virtual bool eventFilter( QObject * o, QEvent * e );
    
private:
    Ui::PerspectiveViewWidget *ui;
};

#endif // PERSPECTIVE_VIEW_WIDGET_H
