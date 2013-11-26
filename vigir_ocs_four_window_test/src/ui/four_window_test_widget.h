#ifndef FOUR_WINDOW_TEST_WIDGET_H
#define FOUR_WINDOW_TEST_WIDGET_H

#include <QWidget>

namespace Ui
{
    class FourWindowTestWidget;
}

class FourWindowTestWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit FourWindowTestWidget(QWidget *parent = 0);
    ~FourWindowTestWidget();

    virtual bool eventFilter( QObject * o, QEvent * e );
    
private:
    Ui::FourWindowTestWidget *ui;
};

#endif // FOUR_WINDOW_TEST_WIDGET_H
