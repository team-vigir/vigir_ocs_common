/********************************************************************************
** Form generated from reading UI file 'main_view_widget.ui'
**
** Created: Fri Sep 27 11:19:45 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_VIEW_WIDGET_H
#define UI_MAIN_VIEW_WIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QScrollArea>
#include <QtGui/QSpacerItem>
#include <QtGui/QSplitter>
#include <QtGui/QStackedWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "ui/template_loader_widget.h"

QT_BEGIN_NAMESPACE

class Ui_MainViewWidget
{
public:
    QHBoxLayout *horizontalLayout_2;
    QStackedWidget *stackedWidget;
    QWidget *page;
    QHBoxLayout *horizontalLayout_6;
    QWidget *center_parent_;
    QWidget *page_2;
    QHBoxLayout *horizontalLayout_7;
    QWidget *splitter1;
    QVBoxLayout *verticalLayout_7;
    QSplitter *splitter_3;
    QWidget *widget_3;
    QHBoxLayout *horizontalLayout_5;
    QSplitter *splitter;
    QWidget *top_left_parent_;
    QWidget *top_right_parent_;
    QWidget *widget_4;
    QHBoxLayout *horizontalLayout_4;
    QSplitter *splitter_2;
    QWidget *bottom_left_parent_;
    QWidget *bottom_right_parent_;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_6;
    QWidget *ui_control;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_2;
    QRadioButton *camera_tool;
    QRadioButton *widget_tool;
    QRadioButton *template_tool;
    QPushButton *request_point_cloud_;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_3;
    QPushButton *insert_waypoint;
    QPushButton *footstep_pose;
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout_4;
    TemplateLoaderWidget *template_widget;
    QGroupBox *groupBox_4;
    QVBoxLayout *verticalLayout_5;
    QCheckBox *robot_model_2;
    QCheckBox *simulation_robot;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QCheckBox *grid_map;
    QPushButton *reset_map;
    QCheckBox *footstep_planning;
    QCheckBox *templates;
    QCheckBox *grasp_model;
    QWidget *widget_2;
    QHBoxLayout *horizontalLayout_3;
    QCheckBox *point_cloud_request;
    QPushButton *reset_point_cloud;
    QCheckBox *lidar_point_cloud_2;
    QCheckBox *stereo_point_cloud_2;
    QCheckBox *laser_scan_2;
    QCheckBox *octomap_2;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *MainViewWidget)
    {
        if (MainViewWidget->objectName().isEmpty())
            MainViewWidget->setObjectName(QString::fromUtf8("MainViewWidget"));
        MainViewWidget->resize(942, 676);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainViewWidget->sizePolicy().hasHeightForWidth());
        MainViewWidget->setSizePolicy(sizePolicy);
        MainViewWidget->setContextMenuPolicy(Qt::CustomContextMenu);
        horizontalLayout_2 = new QHBoxLayout(MainViewWidget);
        horizontalLayout_2->setSpacing(9);
        horizontalLayout_2->setContentsMargins(9, 9, 9, 9);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setSizeConstraint(QLayout::SetDefaultConstraint);
        stackedWidget = new QStackedWidget(MainViewWidget);
        stackedWidget->setObjectName(QString::fromUtf8("stackedWidget"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(stackedWidget->sizePolicy().hasHeightForWidth());
        stackedWidget->setSizePolicy(sizePolicy1);
        page = new QWidget();
        page->setObjectName(QString::fromUtf8("page"));
        horizontalLayout_6 = new QHBoxLayout(page);
        horizontalLayout_6->setSpacing(0);
        horizontalLayout_6->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        center_parent_ = new QWidget(page);
        center_parent_->setObjectName(QString::fromUtf8("center_parent_"));
        sizePolicy1.setHeightForWidth(center_parent_->sizePolicy().hasHeightForWidth());
        center_parent_->setSizePolicy(sizePolicy1);

        horizontalLayout_6->addWidget(center_parent_);

        stackedWidget->addWidget(page);
        page_2 = new QWidget();
        page_2->setObjectName(QString::fromUtf8("page_2"));
        horizontalLayout_7 = new QHBoxLayout(page_2);
        horizontalLayout_7->setSpacing(0);
        horizontalLayout_7->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        splitter1 = new QWidget(page_2);
        splitter1->setObjectName(QString::fromUtf8("splitter1"));
        sizePolicy1.setHeightForWidth(splitter1->sizePolicy().hasHeightForWidth());
        splitter1->setSizePolicy(sizePolicy1);
        verticalLayout_7 = new QVBoxLayout(splitter1);
        verticalLayout_7->setSpacing(0);
        verticalLayout_7->setContentsMargins(0, 0, 0, 0);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        splitter_3 = new QSplitter(splitter1);
        splitter_3->setObjectName(QString::fromUtf8("splitter_3"));
        splitter_3->setOrientation(Qt::Vertical);
        widget_3 = new QWidget(splitter_3);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        horizontalLayout_5 = new QHBoxLayout(widget_3);
        horizontalLayout_5->setSpacing(0);
        horizontalLayout_5->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        splitter = new QSplitter(widget_3);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        top_left_parent_ = new QWidget(splitter);
        top_left_parent_->setObjectName(QString::fromUtf8("top_left_parent_"));
        sizePolicy1.setHeightForWidth(top_left_parent_->sizePolicy().hasHeightForWidth());
        top_left_parent_->setSizePolicy(sizePolicy1);
        splitter->addWidget(top_left_parent_);
        top_right_parent_ = new QWidget(splitter);
        top_right_parent_->setObjectName(QString::fromUtf8("top_right_parent_"));
        sizePolicy1.setHeightForWidth(top_right_parent_->sizePolicy().hasHeightForWidth());
        top_right_parent_->setSizePolicy(sizePolicy1);
        splitter->addWidget(top_right_parent_);

        horizontalLayout_5->addWidget(splitter);

        splitter_3->addWidget(widget_3);
        widget_4 = new QWidget(splitter_3);
        widget_4->setObjectName(QString::fromUtf8("widget_4"));
        horizontalLayout_4 = new QHBoxLayout(widget_4);
        horizontalLayout_4->setSpacing(0);
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        splitter_2 = new QSplitter(widget_4);
        splitter_2->setObjectName(QString::fromUtf8("splitter_2"));
        splitter_2->setOrientation(Qt::Horizontal);
        bottom_left_parent_ = new QWidget(splitter_2);
        bottom_left_parent_->setObjectName(QString::fromUtf8("bottom_left_parent_"));
        sizePolicy1.setHeightForWidth(bottom_left_parent_->sizePolicy().hasHeightForWidth());
        bottom_left_parent_->setSizePolicy(sizePolicy1);
        splitter_2->addWidget(bottom_left_parent_);
        bottom_right_parent_ = new QWidget(splitter_2);
        bottom_right_parent_->setObjectName(QString::fromUtf8("bottom_right_parent_"));
        sizePolicy1.setHeightForWidth(bottom_right_parent_->sizePolicy().hasHeightForWidth());
        bottom_right_parent_->setSizePolicy(sizePolicy1);
        bottom_right_parent_->setMouseTracking(true);
        splitter_2->addWidget(bottom_right_parent_);

        horizontalLayout_4->addWidget(splitter_2);

        splitter_3->addWidget(widget_4);

        verticalLayout_7->addWidget(splitter_3);


        horizontalLayout_7->addWidget(splitter1);

        stackedWidget->addWidget(page_2);

        horizontalLayout_2->addWidget(stackedWidget);

        scrollArea = new QScrollArea(MainViewWidget);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setMinimumSize(QSize(280, 0));
        scrollArea->setMaximumSize(QSize(280, 16777215));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 264, 782));
        verticalLayout_6 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(7, 7, 7, 7);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        ui_control = new QWidget(scrollAreaWidgetContents);
        ui_control->setObjectName(QString::fromUtf8("ui_control"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(ui_control->sizePolicy().hasHeightForWidth());
        ui_control->setSizePolicy(sizePolicy2);
        ui_control->setMinimumSize(QSize(250, 0));
        ui_control->setMaximumSize(QSize(250, 16777215));
        ui_control->setBaseSize(QSize(0, 0));
        verticalLayout = new QVBoxLayout(ui_control);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        groupBox = new QGroupBox(ui_control);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setCheckable(true);
        verticalLayout_2 = new QVBoxLayout(groupBox);
        verticalLayout_2->setSpacing(3);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, -1, 0, 0);
        camera_tool = new QRadioButton(groupBox);
        camera_tool->setObjectName(QString::fromUtf8("camera_tool"));
        camera_tool->setChecked(true);

        verticalLayout_2->addWidget(camera_tool);

        widget_tool = new QRadioButton(groupBox);
        widget_tool->setObjectName(QString::fromUtf8("widget_tool"));

        verticalLayout_2->addWidget(widget_tool);

        template_tool = new QRadioButton(groupBox);
        template_tool->setObjectName(QString::fromUtf8("template_tool"));

        verticalLayout_2->addWidget(template_tool);

        request_point_cloud_ = new QPushButton(groupBox);
        request_point_cloud_->setObjectName(QString::fromUtf8("request_point_cloud_"));

        verticalLayout_2->addWidget(request_point_cloud_);


        verticalLayout->addWidget(groupBox);

        groupBox_2 = new QGroupBox(ui_control);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setCheckable(true);
        verticalLayout_3 = new QVBoxLayout(groupBox_2);
        verticalLayout_3->setSpacing(3);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, -1, 0, 0);
        insert_waypoint = new QPushButton(groupBox_2);
        insert_waypoint->setObjectName(QString::fromUtf8("insert_waypoint"));

        verticalLayout_3->addWidget(insert_waypoint);

        footstep_pose = new QPushButton(groupBox_2);
        footstep_pose->setObjectName(QString::fromUtf8("footstep_pose"));

        verticalLayout_3->addWidget(footstep_pose);


        verticalLayout->addWidget(groupBox_2);

        groupBox_3 = new QGroupBox(ui_control);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setCheckable(true);
        verticalLayout_4 = new QVBoxLayout(groupBox_3);
        verticalLayout_4->setSpacing(0);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 9, 0, 0);
        template_widget = new TemplateLoaderWidget(groupBox_3);
        template_widget->setObjectName(QString::fromUtf8("template_widget"));
        template_widget->setMinimumSize(QSize(0, 205));

        verticalLayout_4->addWidget(template_widget);


        verticalLayout->addWidget(groupBox_3);


        verticalLayout_6->addWidget(ui_control);

        groupBox_4 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setFlat(false);
        groupBox_4->setCheckable(true);
        verticalLayout_5 = new QVBoxLayout(groupBox_4);
        verticalLayout_5->setSpacing(3);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, -1, 0, 0);
        robot_model_2 = new QCheckBox(groupBox_4);
        robot_model_2->setObjectName(QString::fromUtf8("robot_model_2"));
        robot_model_2->setChecked(true);

        verticalLayout_5->addWidget(robot_model_2);

        simulation_robot = new QCheckBox(groupBox_4);
        simulation_robot->setObjectName(QString::fromUtf8("simulation_robot"));
        simulation_robot->setChecked(false);

        verticalLayout_5->addWidget(simulation_robot);

        widget = new QWidget(groupBox_4);
        widget->setObjectName(QString::fromUtf8("widget"));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        grid_map = new QCheckBox(widget);
        grid_map->setObjectName(QString::fromUtf8("grid_map"));
        grid_map->setChecked(true);

        horizontalLayout->addWidget(grid_map);

        reset_map = new QPushButton(widget);
        reset_map->setObjectName(QString::fromUtf8("reset_map"));
        reset_map->setMaximumSize(QSize(40, 18));
        reset_map->setStyleSheet(QString::fromUtf8("font: 75 8pt \"Ubuntu\";\n"
"color: rgb(255, 26, 26);"));

        horizontalLayout->addWidget(reset_map);


        verticalLayout_5->addWidget(widget);

        footstep_planning = new QCheckBox(groupBox_4);
        footstep_planning->setObjectName(QString::fromUtf8("footstep_planning"));
        footstep_planning->setChecked(true);

        verticalLayout_5->addWidget(footstep_planning);

        templates = new QCheckBox(groupBox_4);
        templates->setObjectName(QString::fromUtf8("templates"));
        templates->setChecked(true);

        verticalLayout_5->addWidget(templates);

        grasp_model = new QCheckBox(groupBox_4);
        grasp_model->setObjectName(QString::fromUtf8("grasp_model"));
        grasp_model->setChecked(true);

        verticalLayout_5->addWidget(grasp_model);

        widget_2 = new QWidget(groupBox_4);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        horizontalLayout_3 = new QHBoxLayout(widget_2);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        point_cloud_request = new QCheckBox(widget_2);
        point_cloud_request->setObjectName(QString::fromUtf8("point_cloud_request"));
        point_cloud_request->setChecked(true);

        horizontalLayout_3->addWidget(point_cloud_request);

        reset_point_cloud = new QPushButton(widget_2);
        reset_point_cloud->setObjectName(QString::fromUtf8("reset_point_cloud"));
        reset_point_cloud->setMaximumSize(QSize(40, 18));
        reset_point_cloud->setStyleSheet(QString::fromUtf8("font: 75 8pt \"Ubuntu\";\n"
"color: rgb(255, 26, 26);"));

        horizontalLayout_3->addWidget(reset_point_cloud);


        verticalLayout_5->addWidget(widget_2);

        lidar_point_cloud_2 = new QCheckBox(groupBox_4);
        lidar_point_cloud_2->setObjectName(QString::fromUtf8("lidar_point_cloud_2"));
        lidar_point_cloud_2->setChecked(false);

        verticalLayout_5->addWidget(lidar_point_cloud_2);

        stereo_point_cloud_2 = new QCheckBox(groupBox_4);
        stereo_point_cloud_2->setObjectName(QString::fromUtf8("stereo_point_cloud_2"));
        stereo_point_cloud_2->setChecked(false);

        verticalLayout_5->addWidget(stereo_point_cloud_2);

        laser_scan_2 = new QCheckBox(groupBox_4);
        laser_scan_2->setObjectName(QString::fromUtf8("laser_scan_2"));
        laser_scan_2->setChecked(false);

        verticalLayout_5->addWidget(laser_scan_2);

        octomap_2 = new QCheckBox(groupBox_4);
        octomap_2->setObjectName(QString::fromUtf8("octomap_2"));
        octomap_2->setChecked(false);

        verticalLayout_5->addWidget(octomap_2);


        verticalLayout_6->addWidget(groupBox_4);

        verticalSpacer = new QSpacerItem(0, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_6->addItem(verticalSpacer);

        scrollArea->setWidget(scrollAreaWidgetContents);

        horizontalLayout_2->addWidget(scrollArea);


        retranslateUi(MainViewWidget);

        stackedWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainViewWidget);
    } // setupUi

    void retranslateUi(QWidget *MainViewWidget)
    {
        MainViewWidget->setWindowTitle(QApplication::translate("MainViewWidget", "MainViewWidget", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainViewWidget", "Tools", 0, QApplication::UnicodeUTF8));
        camera_tool->setText(QApplication::translate("MainViewWidget", "Camera Manipulation", 0, QApplication::UnicodeUTF8));
        widget_tool->setText(QApplication::translate("MainViewWidget", "Joint Manipulation", 0, QApplication::UnicodeUTF8));
        template_tool->setText(QApplication::translate("MainViewWidget", "Template Manipulation", 0, QApplication::UnicodeUTF8));
        request_point_cloud_->setText(QApplication::translate("MainViewWidget", "Request Point Cloud", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("MainViewWidget", "Navigation", 0, QApplication::UnicodeUTF8));
        insert_waypoint->setText(QApplication::translate("MainViewWidget", "Insert Waypoint", 0, QApplication::UnicodeUTF8));
        footstep_pose->setText(QApplication::translate("MainViewWidget", "Define Target Pose", 0, QApplication::UnicodeUTF8));
        footstep_pose->setShortcut(QApplication::translate("MainViewWidget", "G", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("MainViewWidget", "Template", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("MainViewWidget", "Visibility", 0, QApplication::UnicodeUTF8));
        robot_model_2->setText(QApplication::translate("MainViewWidget", "Robot", 0, QApplication::UnicodeUTF8));
        robot_model_2->setShortcut(QApplication::translate("MainViewWidget", "Q", 0, QApplication::UnicodeUTF8));
        simulation_robot->setText(QApplication::translate("MainViewWidget", "Simulation Robot", 0, QApplication::UnicodeUTF8));
        simulation_robot->setShortcut(QApplication::translate("MainViewWidget", "W", 0, QApplication::UnicodeUTF8));
        grid_map->setText(QApplication::translate("MainViewWidget", "Map", 0, QApplication::UnicodeUTF8));
        reset_map->setText(QApplication::translate("MainViewWidget", "reset", 0, QApplication::UnicodeUTF8));
        footstep_planning->setText(QApplication::translate("MainViewWidget", "Footstep Planning", 0, QApplication::UnicodeUTF8));
        templates->setText(QApplication::translate("MainViewWidget", "Templates", 0, QApplication::UnicodeUTF8));
        grasp_model->setText(QApplication::translate("MainViewWidget", "Grasp", 0, QApplication::UnicodeUTF8));
        point_cloud_request->setText(QApplication::translate("MainViewWidget", "Point Cloud Request", 0, QApplication::UnicodeUTF8));
        reset_point_cloud->setText(QApplication::translate("MainViewWidget", "reset", 0, QApplication::UnicodeUTF8));
        lidar_point_cloud_2->setText(QApplication::translate("MainViewWidget", "LIDAR Point Cloud", 0, QApplication::UnicodeUTF8));
        stereo_point_cloud_2->setText(QApplication::translate("MainViewWidget", "Stereo Point Cloud", 0, QApplication::UnicodeUTF8));
        laser_scan_2->setText(QApplication::translate("MainViewWidget", "Laser Scan", 0, QApplication::UnicodeUTF8));
        octomap_2->setText(QApplication::translate("MainViewWidget", "Octomap", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainViewWidget: public Ui_MainViewWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_VIEW_WIDGET_H
