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
    flags |= Qt::Widget;
    flags |= Qt::Dialog; // //ensure as a dialog box, not a seperate window/tab
    this->setWindowFlags(flags);

    this->hide();
    //make background transparent
    //this->setAttribute(Qt::WA_TranslucentBackground);
    //this->setStyleSheet("background:transparent;");

    connect(ui->confirm_button_,SIGNAL(clicked()), this, SLOT(confirm()));
    ui->confirm_button_->hide();
    confirmed_ = false;
    ui->centralWidget->setMouseTracking( true );
    ui->centralWidget->installEventFilter(this);
}

bool BehaviorNotification::eventFilter(QObject* object,QEvent* event)
{
    if (object == ui->centralWidget)
    {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        if(mouseEvent->button() == 1 && mouseEvent->type() == QEvent::MouseButtonPress)
        {
            if(ui->confirm_button_->isVisible())
                ui->confirm_button_->hide();
            else
                ui->confirm_button_->show();
            return true;
        }

    }
    //propagate
    return QMainWindow::eventFilter(object,event);
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
//    QMessageBox::StandardButton reply;
//    reply = QMessageBox::question(this, "Action Confirmation", "complete?",
//                                    QMessageBox::Yes|QMessageBox::No);
//    if (reply == QMessageBox::Yes)
        confirmed_ = true; // notification is now obselete, can be deleted
}


