#include "ui_behavior_notification.h"
#include "behavior_notification.h"

BehaviorNotification::BehaviorNotification(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::BehaviorNotification)
{
    ui->setupUi(this);  

    //remove window border and set on top
//    Qt::WindowFlags flags = this->windowFlags();
//    flags |= Qt::WindowStaysOnTopHint;
//    flags |= Qt::FramelessWindowHint;
//    flags |= Qt::Widget;
//    flags |= Qt::Dialog; // //ensure as a dialog box, not a seperate window/tab
//    this->setWindowFlags(flags);

    this->hide();
    //make background transparent
    //this->setAttribute(Qt::WA_TranslucentBackground);
    //this->setStyleSheet("background:transparent;");

    connect(ui->confirm_button_,SIGNAL(clicked()), this, SLOT(confirm()));
    connect(ui->abort_button_,SIGNAL(clicked()), this, SLOT(abort()));


    ui->confirm_button_->hide();
    ui->abort_button_->hide();
    confirmed_ = false;
    ui->central_widget_->setMouseTracking( true );
    ui->central_widget_->installEventFilter(this);

    //this->setStyleSheet();
}

bool BehaviorNotification::eventFilter(QObject* object,QEvent* event)
{
    if (object == ui->central_widget_)
    {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        if(mouseEvent->button() == 1 && mouseEvent->type() == QEvent::MouseButtonPress)
        {
            if(ui->confirm_button_->isVisible())
            {
                ui->confirm_button_->hide();
                ui->abort_button_->hide();
            }
            else
            {
                ui->confirm_button_->show();
                ui->abort_button_->show();
            }
            return true;
        }

    }
    //propagate
    return QWidget::eventFilter(object,event);
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
    confirmed_ = true; // notification is now obselete, can be deleted
    Q_EMIT sendConfirmation(ui->action_label_->text());
}

void BehaviorNotification::abort()
{
    //this action isn't confirmed, but it still needs to be deleted
    confirmed_ = true;
    Q_EMIT sendAbort(ui->action_label_->text());
}


