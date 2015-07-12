#include "ui_behavior_notification.h"
#include "behavior_notification.h"

BehaviorNotification::BehaviorNotification(QWidget *parent, QString action_text, int goal_id, int goal_type):
    //QWidget(parent),
    ui(new Ui::BehaviorNotification)
{
    ui->setupUi(this);

    this->hide();

    object_sub_ = nh_.subscribe("/flor/ocs/object_selection", 5 , &BehaviorNotification::objectSelectCB, this);

    connect(ui->confirm_button_,SIGNAL(clicked()), this, SLOT(confirm()));
    connect(ui->abort_button_,SIGNAL(clicked()), this, SLOT(abort()));

    confirmed_ = false;
    ui->label_widget_->setMouseTracking(true);

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

    //want frameless widget
    Qt::WindowFlags flags = ui->confirmation_widget_->windowFlags();
    flags |= Qt::FramelessWindowHint;
    flags |= Qt::WindowStaysOnTopHint;
    //flags |= Qt::Dialog;
    ui->confirmation_widget_->setWindowFlags(flags);


    timer.start(100, this);

    //dummy init
    main_view_point_ = this->mapToGlobal(this->geometry().topLeft());

    goal_id_ = goal_id;
    goal_type_ = goal_type;

    ui->action_label_->setText(action_text);

    //prevent sending bad data on start
    //only protecting selected object types for now
    if(goal_type_ == vigir_be_msgs::BehaviorInputGoal::SELECTED_OBJECT_ID)
        ui->confirm_button_->setDisabled(true);
}

void BehaviorNotification::timerEvent(QTimerEvent *event)
{
    if(ui->confirm_button_->isVisible())
    {
        QPoint p = QWidget::mapToGlobal(this->geometry().topRight());
                                                    //x works fine from global, but y is messed up, unless grabbed elsewhere
        ui->confirmation_widget_->setGeometry(this->mapToGlobal(this->geometry().topRight()).x(), main_view_point_.y() + 45,//plus height of toolbar, yes this is awkward and shouldn't be getting position from main view
                                              ui->confirmation_widget_->geometry().width(), ui->confirmation_widget_->geometry().height());
    }
}

bool BehaviorNotification::eventFilter(QObject* object, QEvent* event)
{
    if (object == ui->label_widget_)
    {     
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        if(mouseEvent->button() == 1 && mouseEvent->type() == QEvent::MouseButtonPress)
        {
            if(ui->confirm_button_->isVisible())
            {             
                ui->confirmation_widget_->setVisible(false);
            }
            else
            {                
                ui->confirmation_widget_->setVisible(true);
            }
            return true;
        }

    }           
    //propagate
    return QWidget::eventFilter(object, event);
}


BehaviorNotification::~BehaviorNotification()
{    
    delete ui;
    //delete goal_;
}

void BehaviorNotification::objectSelectCB(const vigir_ocs_msgs::OCSObjectSelection::ConstPtr msg)
{    
    //don't want to be affected by selection by another operator
    if(msg->host == boost::asio::ip::host_name())
    {
        //ignore this message if our notification isn't meant for selected objects
        if(goal_type_ == vigir_be_msgs::BehaviorInputGoal::SELECTED_OBJECT_ID)
        {
            //must have template selected to confirm, other selections must turn of confirm for safety
            if(msg->type == vigir_ocs_msgs::OCSObjectSelection::TEMPLATE)
            {
                ui->confirm_button_->setDisabled(false);
            }
            else
            {
                ui->confirm_button_->setDisabled(true);
            }
        }
    }
}

void BehaviorNotification::setActionText(QString action_text)
{
    ui->action_label_->setText(action_text);
}

int BehaviorNotification::getGoalId()
{
    return goal_id_;
}

bool BehaviorNotification::getConfirmed()
{
    return confirmed_;
}

void BehaviorNotification::setPoint(QPoint point)
{
    main_view_point_ = point;
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
                                                "QPushButton:!enabled{"+
                                                " background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(30, 30, 30, 255), stop:1 rgba(10, 10, 10, 255));" +
                                                " color: rgb(120, 120, 120);" +
                                                "}"+
                                                "QPushButton:pressed  {" +
                                                " padding-top:1px; padding-left:1px;" +
                                                " background-color: rgb(30,30,30);" +
                                                " border-style: inset;" +
                                                "}");
}


void BehaviorNotification::confirm()
{    
    //set confirmation to be parented to this so it is deleted with rest of notification
    ui->confirmation_widget_->setParent(this);

    Q_EMIT sendConfirmation(ui->action_label_->text(), goal_id_);
}

void BehaviorNotification::abort()
{   
    //set confirmation to be parented to this so it is deleted with rest of notification
    ui->confirmation_widget_->setParent(this);

    Q_EMIT sendAbort(ui->action_label_->text(), goal_id_);
}

//delete from ui without confirming or aborting action
void BehaviorNotification::queueDeleteNotification()
{
    ROS_ERROR("delete queued");
    //will be cleaned up on next cleanNotification call from relay
    confirmed_ = true;
}
