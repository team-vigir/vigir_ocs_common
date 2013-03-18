#include "template_manager_widget.h"
#include "ui_template_manager_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>

TemplateManagerWidget::TemplateManagerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TemplateManagerWidget),
    templateDirPath(QString("/opt/vigir/rosbuild_ws/vigir_control/vigir_grasping/templates/")) // should read this from file
{
    ui->setupUi(this);

    // load all directories under the templates directory and all .mesh files
    QDir* rootDir = new QDir(templateDirPath);

    {
        QStringList nameFilter;
        QFileInfoList filesList = rootDir->entryInfoList(nameFilter, QDir::Dirs);

        for(int i = 0; i < filesList.size(); i++)
        {
            QFileInfo fileInfo = filesList[i];

            if( fileInfo.fileName().compare(".") != 0 && fileInfo.fileName().compare("..") != 0 )
            {
                QTreeWidgetItem* item = new QTreeWidgetItem();
                item->setText(0,fileInfo.fileName());

                if(fileInfo.isDir())
                {
                    addTreeWidgetChild(item);
                }

                ui->treeWidget->addTopLevelItem(item);
            }
        }
    }

    ui->treeWidget->expandAll();
}

TemplateManagerWidget::~TemplateManagerWidget()
{
    delete ui;
}

// since we always have 1 for directory depth, we won't do anything fancy other than looking for .mesh files
void TemplateManagerWidget::addTreeWidgetChild(QTreeWidgetItem* item)
{
    QDir* rootDir = new QDir(templateDirPath+item->text(0));
    QStringList nameFilter;
    nameFilter << "*.mesh";
    QFileInfoList filesList = rootDir->entryInfoList(nameFilter, QDir::Files);

    for(int i = 0; i < filesList.size(); i++)
    {
        QFileInfo fileInfo = filesList[i];

        QTreeWidgetItem* child = new QTreeWidgetItem();
        child->setText(0,fileInfo.fileName());

        item->addChild(child);

        ui->treeWidget->resizeColumnToContents(0);
    }
}

void TemplateManagerWidget::treeItemChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous)
{
    if(current->parent()) // it means that it is a sub-item/mesh file
    {
        //std::cout << current->parent()->text(0).toStdString() << std::endl;
        //std::cout << " -> " << current->text(0).toStdString() << std::endl;

        // try to load the .png file with same name as the mesh file
        QString imageName = current->text(0).remove(".mesh");
        QString url = templateDirPath+current->parent()->text(0)+"/"+imageName+".png";

        ui->widget->setStyleSheet(QString("image:url('")+url+QString("');"));

        templatePath = current->parent()->text(0)+"/"+current->text(0);
    }
    else // main item, just clear
    {
        ui->widget->setStyleSheet(QString(""));

        templatePath = "";
    }
}

void TemplateManagerWidget::insertButtonPressed()
{
    if(templatePath != "")
    {
        Q_EMIT insertTemplate(templatePath);
    }
}
