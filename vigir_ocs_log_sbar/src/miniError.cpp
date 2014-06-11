
#include "ui_miniError.h"
#include "miniError.h"


MiniError::MiniError(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MiniError)
{
    ui->setupUi(this);

    timer = new QTimer(this);

    robStatus = new robotStatus();
    //connect signals to retrieve error data
    connect(robStatus,SIGNAL(sendErrorData(QString,QString)),this,SLOT(receiveErrorData(QString, QString)));
    connect(robStatus,SIGNAL(sendErrorData(QString,QString)),this->parent(),SLOT(receiveErrorData(QString, QString)));
    //
    connect(this->parent(),SIGNAL(makeErrorActive()),this,SLOT(startActiveTimer()));

    //connect(ui->table,SIGNAL(cellClicked(int,int)),this,SLOT(mouseClick(int,int)));
    //connect(ui->table,SIGNAL(itemClicked(QTableWidgetItem)),this,SLOT(mouseClick(QTableWidgetItem)));
    ui->table->setColumnWidth(0,100);
    ui->table->setColumnWidth(1,300);

    //build animations
    errorFadeIn = new QPropertyAnimation(this, "windowOpacity");
    errorFadeIn->setEasingCurve(QEasingCurve::InOutQuad);
    errorFadeIn->setDuration(500);
    errorFadeIn->setStartValue(0.0);
    errorFadeIn->setEndValue(.75);

    errorFadeOut = new QPropertyAnimation(this, "windowOpacity");
    errorFadeOut->setEasingCurve(QEasingCurve::InOutQuad);
    errorFadeOut->setDuration(300);
    errorFadeOut->setStartValue(0.75);
    errorFadeOut->setEndValue(0.0);

    newErrors = 0;

    //originalGeometry

    visible = false;
}

void MiniError::receiveErrorData(QString time, QString message)
{    
    //used for insertions
    simTime = new QTableWidgetItem();
    errorMessage = new QTableWidgetItem();

    //insert new row on new error
    ui->table->insertRow(0);
    simTime->setText(time);
    simTime->setBackgroundColor(Qt::red);
    errorMessage->setText(message);
    errorMessage->setBackgroundColor(Qt::red);
    ui->table->setItem(0,0,simTime);
    ui->table->setItem(0,1,errorMessage);
    ui->table->setRowCount(5);// only last 5 errors maximum
    newErrors++;
}

void MiniError::enterEvent(QEvent * event)
{
    if(timer->isActive())
    {
        this->show();
        errorFadeIn->start();
        visible = true;
    }

}

void MiniError::leaveEvent(QEvent * event)
{
    if(visible)
    {
        this->hide();
        errorFadeOut->start();
    }
    //set errors as viewed
    for(int i=0;i<newErrors;i++)
    {
        ui->table->item(i,0)->setBackgroundColor(Qt::white);
        ui->table->item(i,1)->setBackgroundColor(Qt::white);
    }
    newErrors = 0;
    visible = false;
    timer->stop(); //ensure stopped
}

void MiniError::startActiveTimer()
{
    timer->start(500);
}

MiniError::~MiniError()
{
    delete(simTime);
    delete(errorMessage);
    delete(errorFadeIn);
    delete(errorFadeOut);
    delete ui;
}


