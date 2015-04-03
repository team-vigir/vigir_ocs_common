#include "ui_behavior_notification.h"
#include "behavior_notification.h"

BehaviorNotification::BehaviorNotification(QWidget *parent) :
    //QWidget(parent),
    ui(new Ui::BehaviorNotification)
{
    ui->setupUi(this);

    this->hide();

    connect(ui->confirm_button_,SIGNAL(clicked()), this, SLOT(confirm()));
    connect(ui->abort_button_,SIGNAL(clicked()), this, SLOT(abort()));

    ui->confirm_button_->hide();
    ui->abort_button_->hide();
    confirmed_ = false;
    ui->central_widget_->setMouseTracking( true );
    ui->central_widget_->installEventFilter(this);

    this->setStyleSheet("background-color:rgba(255,255,255,200);");

}

bool BehaviorNotification::eventFilter(QObject* object,QEvent* event)
{
    if (object == ui->central_widget_)
    {
        //lighten as hovered over
        //this->setStyleSheet("background-color:rgba(235,235,235,200);");
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
    //reset color
    //this->setStyleSheet("background-color:rgba(255,255,255,200);");
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

void BehaviorNotification::setGoal(BehaviorServer::GoalHandlePtr goal_handle)
{
    //not great to pass goal along with notification, but im going to live with it for now
    goal_ = goal_handle;
}



void BehaviorNotification::confirm()
{
    confirmed_ = true; // notification is now obselete, can be deleted
    Q_EMIT sendConfirmation(ui->action_label_->text(),goal_);
}

void BehaviorNotification::abort()
{
    //this action isn't confirmed, but it still needs to be deleted
    confirmed_ = true;
    Q_EMIT sendAbort(ui->action_label_->text(),goal_);
}
