#ifndef DisplayOptionsWidget_H
#define DisplayOptionsWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>

#include <QPainter>
#include <QtGui>


namespace Ui {
class DisplayOptionsWidget;
}

class DisplayOptionsWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit DisplayOptionsWidget(QWidget *parent = 0);
    ~DisplayOptionsWidget();

private:
    Ui::DisplayOptionsWidget* ui;

};

#endif // DisplayOptionsWidget_H
