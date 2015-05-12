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

    //ui->confirm_button_->hide();
    //ui->abort_button_->hide();
    confirmed_ = false;
    ui->central_widget_->setMouseTracking( true );
    ui->central_widget_->setWindowOpacity(0);
    ui->central_widget_->setAttribute(Qt::WA_TranslucentBackground);
    ui->central_widget_->setStyleSheet("background:transparent;");

    ui->central_widget_->installEventFilter(this);

    ui->confirmation_widget_->hide();

    this->setStyleSheet("QWidget{background-color:rgba(255,255,255,200);}"
                        "QWidget:hover{background-color:rgba(235,235,235,200);}"
                        );

    setButtonStyle(ui->confirm_button_);
    setButtonStyle(ui->abort_button_);


    confirm_fadein_ = new QPropertyAnimation(ui->confirmation_widget_, "windowOpacity");
    confirm_fadein_->setTargetObject(ui->confirmation_widget_);
    confirm_fadein_->setEasingCurve(QEasingCurve::InOutQuad);
    confirm_fadein_->setDuration(500);
    confirm_fadein_->setStartValue(0.0);
    confirm_fadein_->setEndValue(.8);

//    QFrame* leftFrame = new QFrame(ui->central_widget_);
//    leftFrame->setLayout(ui->central_widget_->layout());
//    leftFrame->setFrameStyle(QFrame::Panel| QFrame::Plain);
//    leftFrame->setLineWidth(2);
//    leftFrame->setObjectName("leftFrame");
//    leftFrame->setStyleSheet("#leftFrame {color: green;}");


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
                ui->confirmation_widget_->hide();
            }
            else
            {
                ui->confirmation_widget_->show();
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
                                                " background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(240, 240, 240, 255), stop:1 rgba(222, 222, 222, 255));" +
                                                " border-style: solid;" +
                                                " border-width: 1px;" +
                                                " border-radius: 1px;" +
                                                " border-color: gray;" +
                                                " padding: 0px;" +
                                                "}" +
                                                "QPushButton:pressed  {" +
                                                " padding-top:1px; padding-left:1px;" +
                                                " background-color: rgb(180,180,180);" +
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
