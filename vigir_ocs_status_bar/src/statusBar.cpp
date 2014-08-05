
#include "ui_statusBar.h"
#include "statusBar.h"


StatusBar::StatusBar(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::StatusBar)
{
    ui->setupUi(this);
    glanceSbar = new glancehubSbar(this);
    logSbar = new LogSbar(this);

    ui->errorLayout->addWidget(logSbar);    

    ui->glanceLayout->addWidget(glanceSbar);

    ui->positionLabel->setText("None Selected");
}


void StatusBar::receivePositionText(QString s)
{
    ui->positionLabel->setText(s);
}

void StatusBar::receiveFPS(int fps)
{
    ui->fpsLabel->setText(QString::number(fps));
}

StatusBar::~StatusBar()
{
    delete ui;
}
