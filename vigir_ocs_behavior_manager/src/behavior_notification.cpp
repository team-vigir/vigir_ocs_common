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

    this->show();
    //make background transparent
    //this->setAttribute(Qt::WA_TranslucentBackground);
    //this->setStyleSheet("background:transparent;");

    //ui->action_label_->setText(action_text);

    connect(ui->confirm_button_,SIGNAL(clicked()), this, SLOT(confirm()));
    confirmed_ = false;
}

BehaviorNotification::~BehaviorNotification()
{
    delete ui;
}

void BehaviorNotification::setActionText(QString action_text)
{
    ui->action_label_->setText(action_text);
}

void BehaviorNotification::confirm()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Action Confirmation", "Action Complete?",
                                    QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes)
        confirmed_ = true; // notification is now obselete, can be deleted
}


