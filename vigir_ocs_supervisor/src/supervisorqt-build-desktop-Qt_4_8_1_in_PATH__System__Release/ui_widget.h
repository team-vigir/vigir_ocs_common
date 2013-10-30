/********************************************************************************
** Form generated from reading UI file 'widget.ui'
**
** Created: Wed Sep 18 14:15:06 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WIDGET_H
#define UI_WIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTableWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Widget
{
public:
    QWidget *widget_4;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_5;
    QLabel *robo_st;
    QLineEdit *r_state;
    QHBoxLayout *horizontalLayout_2;
    QLabel *curst;
    QLineEdit *cur_st;
    QHBoxLayout *horizontalLayout_6;
    QLabel *d_label;
    QLineEdit *d_state;
    QHBoxLayout *horizontalLayout_12;
    QLabel *rfault;
    QLineEdit *fault;
    QWidget *widget_5;
    QLabel *last_stat;
    QTableWidget *stat;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_6;
    QWidget *widget;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *connect;
    QSpacerItem *horizontalSpacer_3;
    QWidget *widget_2;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_3;
    QLabel *pr;
    QHBoxLayout *horizontalLayout;
    QRadioButton *off;
    QRadioButton *low;
    QRadioButton *high;
    QSpacerItem *horizontalSpacer_2;
    QWidget *widget_7;
    QHBoxLayout *horizontalLayout_11;
    QPushButton *start;
    QSpacerItem *horizontalSpacer_4;
    QWidget *widget_3;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_4;
    QLabel *cs;
    QSpacerItem *horizontalSpacer;
    QListWidget *cs_list;
    QPushButton *send_mode;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_7;
    QWidget *widget_6;
    QLabel *param;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLineEdit *timemeter;
    QLineEdit *supply;
    QLabel *psupply;
    QLineEdit *sump;
    QLabel *pmdt;
    QLabel *prpm;
    QLabel *psump;
    QLineEdit *mt;
    QLabel *preturn;
    QLineEdit *rpm;
    QLabel *ptimemeter;
    QLineEdit *inlet;
    QLabel *ppst;
    QLabel *pmt;
    QLineEdit *pst;
    QLineEdit *return_2;
    QLineEdit *mdt;
    QLabel *pinlet;

    void setupUi(QWidget *Widget)
    {
        if (Widget->objectName().isEmpty())
            Widget->setObjectName(QString::fromUtf8("Widget"));
        Widget->resize(500, 861);
        Widget->setMinimumSize(QSize(10, 10));
        Widget->setMaximumSize(QSize(500, 1000));
        widget_4 = new QWidget(Widget);
        widget_4->setObjectName(QString::fromUtf8("widget_4"));
        widget_4->setGeometry(QRect(10, 340, 451, 152));
        widget_4->setMaximumSize(QSize(500, 16777215));
        verticalLayout_2 = new QVBoxLayout(widget_4);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        robo_st = new QLabel(widget_4);
        robo_st->setObjectName(QString::fromUtf8("robo_st"));

        horizontalLayout_5->addWidget(robo_st);

        r_state = new QLineEdit(widget_4);
        r_state->setObjectName(QString::fromUtf8("r_state"));

        horizontalLayout_5->addWidget(r_state);


        verticalLayout_2->addLayout(horizontalLayout_5);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        curst = new QLabel(widget_4);
        curst->setObjectName(QString::fromUtf8("curst"));

        horizontalLayout_2->addWidget(curst);

        cur_st = new QLineEdit(widget_4);
        cur_st->setObjectName(QString::fromUtf8("cur_st"));

        horizontalLayout_2->addWidget(cur_st);


        verticalLayout_2->addLayout(horizontalLayout_2);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        d_label = new QLabel(widget_4);
        d_label->setObjectName(QString::fromUtf8("d_label"));

        horizontalLayout_6->addWidget(d_label);

        d_state = new QLineEdit(widget_4);
        d_state->setObjectName(QString::fromUtf8("d_state"));

        horizontalLayout_6->addWidget(d_state);


        verticalLayout_2->addLayout(horizontalLayout_6);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        rfault = new QLabel(widget_4);
        rfault->setObjectName(QString::fromUtf8("rfault"));

        horizontalLayout_12->addWidget(rfault);

        fault = new QLineEdit(widget_4);
        fault->setObjectName(QString::fromUtf8("fault"));

        horizontalLayout_12->addWidget(fault);


        verticalLayout_2->addLayout(horizontalLayout_12);

        widget_5 = new QWidget(Widget);
        widget_5->setObjectName(QString::fromUtf8("widget_5"));
        widget_5->setGeometry(QRect(10, 490, 451, 121));
        widget_5->setMaximumSize(QSize(500, 16777215));
        last_stat = new QLabel(widget_5);
        last_stat->setObjectName(QString::fromUtf8("last_stat"));
        last_stat->setGeometry(QRect(1, 1, 156, 17));
        stat = new QTableWidget(widget_5);
        if (stat->columnCount() < 5)
            stat->setColumnCount(5);
        if (stat->rowCount() < 5)
            stat->setRowCount(5);
        stat->setObjectName(QString::fromUtf8("stat"));
        stat->setGeometry(QRect(0, 20, 451, 91));
        stat->setRowCount(5);
        stat->setColumnCount(5);
        layoutWidget = new QWidget(Widget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(9, 9, 268, 328));
        verticalLayout_6 = new QVBoxLayout(layoutWidget);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(0, 0, 0, 0);
        widget = new QWidget(layoutWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        verticalLayout_3 = new QVBoxLayout(widget);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        connect = new QPushButton(widget);
        connect->setObjectName(QString::fromUtf8("connect"));

        horizontalLayout_7->addWidget(connect);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_3);


        verticalLayout_3->addLayout(horizontalLayout_7);


        verticalLayout_6->addWidget(widget);

        widget_2 = new QWidget(layoutWidget);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        verticalLayout_4 = new QVBoxLayout(widget_2);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        pr = new QLabel(widget_2);
        pr->setObjectName(QString::fromUtf8("pr"));
        pr->setMinimumSize(QSize(5, 5));

        horizontalLayout_3->addWidget(pr);


        verticalLayout_4->addLayout(horizontalLayout_3);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        off = new QRadioButton(widget_2);
        off->setObjectName(QString::fromUtf8("off"));
        off->setCheckable(true);
        off->setChecked(false);

        horizontalLayout->addWidget(off);

        low = new QRadioButton(widget_2);
        low->setObjectName(QString::fromUtf8("low"));

        horizontalLayout->addWidget(low);

        high = new QRadioButton(widget_2);
        high->setObjectName(QString::fromUtf8("high"));

        horizontalLayout->addWidget(high);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);


        verticalLayout_4->addLayout(horizontalLayout);


        verticalLayout_6->addWidget(widget_2);

        widget_7 = new QWidget(layoutWidget);
        widget_7->setObjectName(QString::fromUtf8("widget_7"));
        horizontalLayout_11 = new QHBoxLayout(widget_7);
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        start = new QPushButton(widget_7);
        start->setObjectName(QString::fromUtf8("start"));

        horizontalLayout_11->addWidget(start);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_11->addItem(horizontalSpacer_4);


        verticalLayout_6->addWidget(widget_7);

        widget_3 = new QWidget(layoutWidget);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        verticalLayout = new QVBoxLayout(widget_3);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        cs = new QLabel(widget_3);
        cs->setObjectName(QString::fromUtf8("cs"));

        horizontalLayout_4->addWidget(cs);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer);


        verticalLayout->addLayout(horizontalLayout_4);

        cs_list = new QListWidget(widget_3);
        new QListWidgetItem(cs_list);
        new QListWidgetItem(cs_list);
        new QListWidgetItem(cs_list);
        cs_list->setObjectName(QString::fromUtf8("cs_list"));
        cs_list->setMaximumSize(QSize(150, 70));

        verticalLayout->addWidget(cs_list);

        send_mode = new QPushButton(widget_3);
        send_mode->setObjectName(QString::fromUtf8("send_mode"));
        send_mode->setMaximumSize(QSize(150, 16777215));

        verticalLayout->addWidget(send_mode);


        verticalLayout_6->addWidget(widget_3);

        layoutWidget1 = new QWidget(Widget);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(0, 0, 2, 2));
        verticalLayout_7 = new QVBoxLayout(layoutWidget1);
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setContentsMargins(11, 11, 11, 11);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        verticalLayout_7->setContentsMargins(0, 0, 0, 0);
        widget_6 = new QWidget(Widget);
        widget_6->setObjectName(QString::fromUtf8("widget_6"));
        widget_6->setGeometry(QRect(0, 620, 521, 321));
        param = new QLabel(widget_6);
        param->setObjectName(QString::fromUtf8("param"));
        param->setGeometry(QRect(0, 10, 146, 17));
        gridLayoutWidget = new QWidget(widget_6);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(0, 30, 501, 171));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        timemeter = new QLineEdit(gridLayoutWidget);
        timemeter->setObjectName(QString::fromUtf8("timemeter"));
        timemeter->setMinimumSize(QSize(100, 25));
        timemeter->setMaximumSize(QSize(100, 25));

        gridLayout->addWidget(timemeter, 4, 3, 1, 1);

        supply = new QLineEdit(gridLayoutWidget);
        supply->setObjectName(QString::fromUtf8("supply"));
        supply->setMinimumSize(QSize(100, 25));
        supply->setMaximumSize(QSize(100, 25));

        gridLayout->addWidget(supply, 2, 1, 1, 1);

        psupply = new QLabel(gridLayoutWidget);
        psupply->setObjectName(QString::fromUtf8("psupply"));

        gridLayout->addWidget(psupply, 2, 0, 1, 1);

        sump = new QLineEdit(gridLayoutWidget);
        sump->setObjectName(QString::fromUtf8("sump"));
        sump->setMinimumSize(QSize(100, 25));
        sump->setMaximumSize(QSize(100, 25));

        gridLayout->addWidget(sump, 2, 3, 1, 1);

        pmdt = new QLabel(gridLayoutWidget);
        pmdt->setObjectName(QString::fromUtf8("pmdt"));

        gridLayout->addWidget(pmdt, 5, 0, 1, 1);

        prpm = new QLabel(gridLayoutWidget);
        prpm->setObjectName(QString::fromUtf8("prpm"));

        gridLayout->addWidget(prpm, 4, 0, 1, 1);

        psump = new QLabel(gridLayoutWidget);
        psump->setObjectName(QString::fromUtf8("psump"));

        gridLayout->addWidget(psump, 2, 2, 1, 1);

        mt = new QLineEdit(gridLayoutWidget);
        mt->setObjectName(QString::fromUtf8("mt"));
        mt->setMinimumSize(QSize(100, 25));
        mt->setMaximumSize(QSize(100, 25));

        gridLayout->addWidget(mt, 3, 1, 1, 1);

        preturn = new QLabel(gridLayoutWidget);
        preturn->setObjectName(QString::fromUtf8("preturn"));

        gridLayout->addWidget(preturn, 1, 2, 1, 1);

        rpm = new QLineEdit(gridLayoutWidget);
        rpm->setObjectName(QString::fromUtf8("rpm"));
        rpm->setMinimumSize(QSize(100, 25));
        rpm->setMaximumSize(QSize(100, 25));

        gridLayout->addWidget(rpm, 4, 1, 1, 1);

        ptimemeter = new QLabel(gridLayoutWidget);
        ptimemeter->setObjectName(QString::fromUtf8("ptimemeter"));

        gridLayout->addWidget(ptimemeter, 4, 2, 1, 1);

        inlet = new QLineEdit(gridLayoutWidget);
        inlet->setObjectName(QString::fromUtf8("inlet"));
        inlet->setMinimumSize(QSize(100, 25));
        inlet->setMaximumSize(QSize(100, 25));

        gridLayout->addWidget(inlet, 1, 1, 1, 1);

        ppst = new QLabel(gridLayoutWidget);
        ppst->setObjectName(QString::fromUtf8("ppst"));

        gridLayout->addWidget(ppst, 3, 2, 1, 1);

        pmt = new QLabel(gridLayoutWidget);
        pmt->setObjectName(QString::fromUtf8("pmt"));

        gridLayout->addWidget(pmt, 3, 0, 1, 1);

        pst = new QLineEdit(gridLayoutWidget);
        pst->setObjectName(QString::fromUtf8("pst"));
        pst->setMinimumSize(QSize(100, 25));
        pst->setMaximumSize(QSize(100, 25));

        gridLayout->addWidget(pst, 3, 3, 1, 1);

        return_2 = new QLineEdit(gridLayoutWidget);
        return_2->setObjectName(QString::fromUtf8("return_2"));
        return_2->setMinimumSize(QSize(100, 25));
        return_2->setMaximumSize(QSize(100, 25));

        gridLayout->addWidget(return_2, 1, 3, 1, 1);

        mdt = new QLineEdit(gridLayoutWidget);
        mdt->setObjectName(QString::fromUtf8("mdt"));
        mdt->setMinimumSize(QSize(100, 25));
        mdt->setMaximumSize(QSize(100, 25));

        gridLayout->addWidget(mdt, 5, 1, 1, 1);

        pinlet = new QLabel(gridLayoutWidget);
        pinlet->setObjectName(QString::fromUtf8("pinlet"));

        gridLayout->addWidget(pinlet, 1, 0, 1, 1);

        layoutWidget->raise();
        layoutWidget->raise();
        widget_4->raise();
        widget_5->raise();
        widget_6->raise();

        retranslateUi(Widget);

        QMetaObject::connectSlotsByName(Widget);
    } // setupUi

    void retranslateUi(QWidget *Widget)
    {
        Widget->setWindowTitle(QApplication::translate("Widget", "Widget", 0, QApplication::UnicodeUTF8));
        robo_st->setText(QApplication::translate("Widget", "ROBOT RUN STATE:", 0, QApplication::UnicodeUTF8));
        curst->setText(QApplication::translate("Widget", "CURRENT BEHAVIOR", 0, QApplication::UnicodeUTF8));
        d_label->setText(QApplication::translate("Widget", "DESIRED BEHAVIOR", 0, QApplication::UnicodeUTF8));
        rfault->setText(QApplication::translate("Widget", "ROBOT FAULT", 0, QApplication::UnicodeUTF8));
        last_stat->setText(QApplication::translate("Widget", "LAST STATUS MESSAGE", 0, QApplication::UnicodeUTF8));
        connect->setText(QApplication::translate("Widget", "CONNECT", 0, QApplication::UnicodeUTF8));
        pr->setText(QApplication::translate("Widget", "SELECT A PUMP PRESSURE", 0, QApplication::UnicodeUTF8));
        off->setText(QApplication::translate("Widget", "OFF", 0, QApplication::UnicodeUTF8));
        low->setText(QApplication::translate("Widget", "LOW", 0, QApplication::UnicodeUTF8));
        high->setText(QApplication::translate("Widget", "HIGH", 0, QApplication::UnicodeUTF8));
        start->setText(QApplication::translate("Widget", "START", 0, QApplication::UnicodeUTF8));
        cs->setText(QApplication::translate("Widget", "CONTROL STATE", 0, QApplication::UnicodeUTF8));

        const bool __sortingEnabled = cs_list->isSortingEnabled();
        cs_list->setSortingEnabled(false);
        QListWidgetItem *___qlistwidgetitem = cs_list->item(0);
        ___qlistwidgetitem->setText(QApplication::translate("Widget", "FREEZE", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem1 = cs_list->item(1);
        ___qlistwidgetitem1->setText(QApplication::translate("Widget", "STAND PREP", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem2 = cs_list->item(2);
        ___qlistwidgetitem2->setText(QApplication::translate("Widget", "STAND", 0, QApplication::UnicodeUTF8));
        cs_list->setSortingEnabled(__sortingEnabled);

        send_mode->setText(QApplication::translate("Widget", "SEND MODE", 0, QApplication::UnicodeUTF8));
        param->setText(QApplication::translate("Widget", "ROBOT PARAMETERS", 0, QApplication::UnicodeUTF8));
        psupply->setText(QApplication::translate("Widget", "PUMP SUPPLY PR.", 0, QApplication::UnicodeUTF8));
        pmdt->setText(QApplication::translate("Widget", "MOTOR DRIVER \n"
"TEMP", 0, QApplication::UnicodeUTF8));
        prpm->setText(QApplication::translate("Widget", "PUMP RPM", 0, QApplication::UnicodeUTF8));
        psump->setText(QApplication::translate("Widget", "AIR SUMP PR.", 0, QApplication::UnicodeUTF8));
        preturn->setText(QApplication::translate("Widget", "PUMP RETURN PR", 0, QApplication::UnicodeUTF8));
        ptimemeter->setText(QApplication::translate("Widget", "PUMP TIME METER", 0, QApplication::UnicodeUTF8));
        ppst->setText(QApplication::translate("Widget", "PUMP SUPPLY TEMP.", 0, QApplication::UnicodeUTF8));
        pmt->setText(QApplication::translate("Widget", "MOTOR TEMP", 0, QApplication::UnicodeUTF8));
        pinlet->setText(QApplication::translate("Widget", "PUMP INLET PR.", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Widget: public Ui_Widget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WIDGET_H
