#ifndef ORTHO_VIEW_WIDGET_H
#define ORTHO_VIEW_WIDGET_H

#include <QWidget>
#include <ortho_view.h>

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

    vigir_ocs::OrthoView* ortho_view_;

};

#endif // ORTHO_VIEW_WIDGET_H
