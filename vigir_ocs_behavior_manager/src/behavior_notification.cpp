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

    confirmed_ = false;
    ui->label_widget_->setMouseTracking( true );


    ui->label_widget_->installEventFilter(this);

    ui->confirmation_widget_->setParent(NULL);

    ui->confirmation_widget_->show();
    ui->confirmation_widget_->setVisible(false);

    ui->label_widget_->setStyleSheet("QWidget{background-color:rgba(60,60,60,200);}"
                        "QWidget:hover{background-color:rgba(30,30,30,200);}");

    setButtonStyle(ui->confirm_button_);
    setButtonStyle(ui->abort_button_);


//    QFrame* leftFrame = new QFrame(ui->central_widget_);
//    leftFrame->setLayout(ui->central_widget_->layout());
//    leftFrame->setFrameStyle(QFrame::Panel| QFrame::Plain);
//    leftFrame->setLineWidth(2);
//    leftFrame->setObjectName("leftFrame");
//    leftFrame->setStyleSheet("#leftFrame {color: green;}");

    Qt::WindowFlags flags = ui->confirmation_widget_->windowFlags();
    //flags |= Qt::WindowStaysOnTopHint;
    flags |= Qt::FramelessWindowHint;
    flags |= Qt::Dialog;
    flags |= Qt::WindowStaysOnTopHint;
    ui->confirmation_widget_->setWindowFlags(flags);

    timer.start(100, this);

}

void BehaviorNotification::timerEvent(QTimerEvent *event)
{
    if(ui->confirm_button_->isVisible())
    {
        QPoint p = QWidget::mapToGlobal(this->geometry().topRight());
        //position to right of rest of notification
        ui->confirmation_widget_->setGeometry(p.x(),p.y(),
                                              ui->confirmation_widget_->geometry().width(),ui->confirmation_widget_->geometry().height());
    }
}

bool BehaviorNotification::eventFilter(QObject* object,QEvent* event)
{
    if (object == ui->label_widget_)
    {     
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        if(mouseEvent->button() == 1 && mouseEvent->type() == QEvent::MouseButtonPress)
        {
            if(ui->confirm_button_->isVisible())
            {
                //ui->confirmation_widget_->hide();
                ui->confirmation_widget_->setVisible(false);
            }
            else
            {
                //ui->confirmation_widget_->show();
                ui->confirmation_widget_->setVisible(true);
               // confirm_fadein_->start();
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
    //delete goal_;
}

void BehaviorNotification::setActionText(QString action_text)
{
    ui->action_label_->setText(action_text);
}

void BehaviorNotification::setGoal(BehaviorServer::GoalHandle goal_handle)
{
    //not great to pass goal along with notification, but im going to live with it for now
    goal_ = goal_handle;
}

void BehaviorNotification::setButtonStyle(QPushButton* btn)
{
    btn->setStyleSheet(QString("QPushButton  { ") +
                                                " background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(60, 60, 60, 255), stop:1 rgba(40, 40, 40, 255));" +
                                                " border-style: solid;" +
                                                " border-width: 1px;" +
                                                " border-radius: 1px;" +
                                                " border-color: gray;" +
                                                " padding: 0px;" +
                                                " color: rgb(255, 255, 255);" +
                                                "}" +
                                                "QPushButton:pressed  {" +
                                                " padding-top:1px; padding-left:1px;" +
                                                " background-color: rgb(30,30,30);" +
                                                " border-style: inset;" +
                                                "}");
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
