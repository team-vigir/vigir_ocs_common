#ifndef ORTHO_VIEW_WIDGET_H
#define ORTHO_VIEW_WIDGET_H

#include <QWidget>

namespace Ui
{
    class OrthoViewWidget;
}

class OrthoViewWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit OrthoViewWidget(QWidget *parent = 0);
    ~OrthoViewWidget();

    virtual bool eventFilter( QObject * o, QEvent * e );
    
private:
    Ui::OrthoViewWidget *ui;

public Q_SLOTS:
    void hideWaypointButton();
    void hideJoystick();
    void requestMap();
    void requestOctomap();

};

#endif // ORTHO_VIEW_WIDGET_H
