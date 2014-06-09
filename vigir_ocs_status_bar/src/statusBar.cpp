
#include "ui_statusBar.h"
#include "statusBar.h"
#include<QFile>
#include<QTextStream>
#include<QDebug>
#include <QSpacerItem>

StatusBar::StatusBar(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::StatusBar)
{
    ui->setupUi(this);
    glanceSbar = new glancehubSbar(this);
    logSbar = new LogSbar(this);

    ui->errorLayout->addWidget(logSbar);
    ui->glanceLayout->addWidget(glanceSbar);

    QSpacerItem* spacer = new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding);
    ui->statusBarLayout->addSpacerItem(spacer);
}


void StatusBar::receivePositionText(QString s)
{
    ui->positionLabel->setText(s);
}

StatusBar::~StatusBar()
{
    delete ui;
}
