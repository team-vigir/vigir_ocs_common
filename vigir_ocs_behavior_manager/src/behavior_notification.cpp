#include "ui_behavior_notification.h"
#include "behavior_notification.h"

BehaviorNotification::BehaviorNotification(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::BehaviorNotification)
{
    ui->setupUi(this);  

    //remove window border and set on top
    Qt::WindowFlags flags = this->windowFlags();
    flags |= Qt::WindowStaysOnTopHint;
    flags |= Qt::FramelessWindowHint;
    flags |= Qt::Dialog; // //ensure as a dialog box, not a seperate window/tab
    this->setWindowFlags(flags);

    //make background transparent
    this->setAttribute(Qt::WA_TranslucentBackground);
    this->setStyleSheet("background:transparent;");

    //ui->action_label.
    //connect(ui->confirm_button_,SIGNAL(clicked()), this, SIGNAL(emergencyStop()));
}

BehaviorNotification::~BehaviorNotification()
{
    delete ui;
}

void BehaviorNotification::setActionText(QString action_text)
{
    ui->action_label_->setText(action_text);
}


