/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
#include "template_loader_widget.h"
#include "ui_template_loader_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>

#include <ros/package.h>

TemplateLoaderWidget::TemplateLoaderWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TemplateLoaderWidget)
{
    // Use "templates" package.  @TODO make this a parameter
    std::string template_path = ros::package::getPath("vigir_template_library")+"/object_templates/";//vigir_grasp_control") + "/../templates/";
    //std::cout << "--------------- <" << template_path << ">\n";
    templateDirPath = QString(template_path.c_str());

    ui->setupUi(this);

    // load all directories under the templates directory and all .mesh files
    QDir* rootDir = new QDir(templateDirPath);
    std::cout << "-------Root-------- <" << rootDir->absolutePath().toStdString() << ">\n";

    {
        QStringList nameFilter;
        QFileInfoList filesList = rootDir->entryInfoList(nameFilter, QDir::Dirs);

        for(int i = 0; i < filesList.size(); i++)
        {
            QFileInfo fileInfo = filesList[i];
            if( fileInfo.fileName().compare(".") != 0 && fileInfo.fileName().compare("..") != 0 &&
                fileInfo.fileName().compare("src") != 0 && fileInfo.fileName().compare("lib") != 0 )
            {
                std::cout << "  " << i << "   <" << fileInfo.fileName().toStdString() << "> - use\n";
                QTreeWidgetItem* item = new QTreeWidgetItem();
                item->setText(0,fileInfo.fileName());

                if(fileInfo.isDir())
                {
                    addTreeWidgetChild(item);
                }

                ui->treeWidget->addTopLevelItem(item);
            }
            else
            {
                std::cout << "  " << i << "   <" << fileInfo.fileName().toStdString() << "> - skip!\n";
            }
        }
    }

    ui->treeWidget->expandAll();
}

TemplateLoaderWidget::~TemplateLoaderWidget()
{
    delete ui;
}

// since we always have 1 for directory depth, we won't do anything fancy other than looking for .mesh files
void TemplateLoaderWidget::addTreeWidgetChild(QTreeWidgetItem* item)
{
    QDir* rootDir = new QDir(templateDirPath+item->text(0));
    QStringList nameFilter;
    nameFilter << "*.mesh";
    QFileInfoList filesList = rootDir->entryInfoList(nameFilter, QDir::Files);

    for(int i = 0; i < filesList.size(); i++)
    {
        QFileInfo fileInfo = filesList[i];
        std::cout << "      " << i << "   <" << fileInfo.fileName().toStdString() << ">\n";

        QTreeWidgetItem* child = new QTreeWidgetItem();
        child->setText(0,fileInfo.fileName());

        item->addChild(child);

        ui->treeWidget->resizeColumnToContents(0);
    }
}

QTreeWidget * TemplateLoaderWidget::getTreeRoot()
{    
    return ui->treeWidget;
}

void TemplateLoaderWidget::treeItemChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous)
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
    Q_EMIT templatePathChanged(templatePath);
}

void TemplateLoaderWidget::insertButtonPressed()
{
    if(templatePath != "")
    {
        Q_EMIT insertTemplate(templatePath);
    }
}
