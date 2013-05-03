#ifndef TemplateLoaderWidget_H
#define TemplateLoaderWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>

#include <QPainter>
#include <QtGui>


namespace Ui {
class TemplateLoaderWidget;
}

class TemplateLoaderWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit TemplateLoaderWidget(QWidget *parent = 0);
    ~TemplateLoaderWidget();
    
private:
    void addTreeWidgetChild(QTreeWidgetItem* item);

    Ui::TemplateLoaderWidget* ui;
    QString templateDirPath;
    QString templatePath;

public Q_SLOTS:
    void treeItemChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous);
    void insertButtonPressed();

Q_SIGNALS:
    void insertTemplate(QString);
    void templatePathChanged(QString);

};

#endif // TemplateLoaderWidget_H
