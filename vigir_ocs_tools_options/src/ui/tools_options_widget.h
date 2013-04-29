#ifndef ToolsOptionsWidget_H
#define ToolsOptionsWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>

#include <QPainter>
#include <QtGui>


namespace Ui {
class ToolsOptionsWidget;
}

class ToolsOptionsWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit ToolsOptionsWidget(QWidget *parent = 0);
    ~ToolsOptionsWidget();

private:
    Ui::ToolsOptionsWidget* ui;

};

#endif // ToolsOptionsWidget_H
