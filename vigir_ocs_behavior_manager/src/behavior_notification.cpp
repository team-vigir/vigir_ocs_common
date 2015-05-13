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
    ui->confirmation_widget_->setStyleSheet("QWidget{background-color:rgba(60,60,60,200);}");
    ui->confirmation_widget_->setMinimumHeight(25);
    ui->confirmation_widget_->setMaximumHeight(25);

    ui->label_widget_->setStyleSheet("QWidget{background-color:rgba(60,60,60,200);}"
                        "QWidget:hover{background-color:rgba(30,30,30,200);}");

    setButtonStyle(ui->confirm_button_);
    setButtonStyle(ui->abort_button_);

    Qt::WindowFlags flags = ui->confirmation_widget_->windowFlags();
    flags |= Qt::FramelessWindowHint;
    //flags |= Qt::Dialog;
    flags |= Qt::WindowStaysOnTopHint;
    ui->confirmation_widget_->setWindowFlags(flags);

    timer.start(100, this);

    //dummy init
    main_view_point_ = this->mapToGlobal(this->geometry().topLeft());

}

void BehaviorNotification::timerEvent(QTimerEvent *event)
{
    if(ui->confirm_button_->isVisible())
    {
        QPoint p = QWidget::mapToGlobal(this->geometry().topRight());
                                                    //x works fine from global, but y is messed up, unless grabbed elsewhere
        ui->confirmation_widget_->setGeometry(this->mapToGlobal(this->geometry().topRight()).x(),main_view_point_.y() + 45,//plus height of toolbar, yes this is awkward and shouldn't be getting position from main view
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

    //set confirmation to be parented to this so it is deleted with rest of notification
    ui->confirmation_widget_->setParent(this);

    Q_EMIT sendConfirmation(ui->action_label_->text(),goal_);
}

void BehaviorNotification::abort()
{
    //this action isn't confirmed, but it still needs to be deleted
    confirmed_ = true;

    //set confirmation to be parented to this so it is deleted with rest of notification
    ui->confirmation_widget_->setParent(this);

    Q_EMIT sendAbort(ui->action_label_->text(),goal_);
}
