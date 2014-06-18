#include "ui_logSbar.h"
#include "logSbar.h"

LogSbar::LogSbar(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::LogSbar)
{
    ui->setupUi(this);

    miniError = new MiniError(this);
    Qt::WindowFlags flags = miniError->windowFlags();
    flags |= Qt::WindowStaysOnTopHint;
    flags |= Qt::FramelessWindowHint;
    flags |= Qt::Dialog; // //ensure ghub as a dialog box, not a seperate window/tab
    miniError->setWindowFlags(flags);
   // miniError->show();
    //miniError->setWindowOpacity(0);

    miniJoint = new MiniJoint(this);
    miniJoint->setWindowFlags(flags);
   // miniJoint->show();
   // miniJoint->setWindowOpacity(0);

    //create animations
    errorFadeIn = new QPropertyAnimation(miniError, "windowOpacity");
    errorFadeIn->setEasingCurve(QEasingCurve::InOutQuad);
    errorFadeIn->setDuration(500);
    errorFadeIn->setStartValue(0.0);
    errorFadeIn->setEndValue(.74);

    errorFadeOut = new QPropertyAnimation(miniError, "windowOpacity");
    errorFadeOut->setEasingCurve(QEasingCurve::InOutQuad);
    errorFadeOut->setDuration(300);
    errorFadeOut->setStartValue(0.74);
    errorFadeOut->setEndValue(0.0);

    jointFadeIn = new QPropertyAnimation(miniJoint, "windowOpacity");
    jointFadeIn->setEasingCurve(QEasingCurve::InOutQuad);
    jointFadeIn->setDuration(500);
    jointFadeIn->setStartValue(0.0);
    jointFadeIn->setEndValue(.74);

    jointFadeOut = new QPropertyAnimation(miniJoint, "windowOpacity");
    jointFadeOut->setEasingCurve(QEasingCurve::InOutQuad);
    jointFadeOut->setDuration(300);
    jointFadeOut->setStartValue(0.74);
    jointFadeOut->setEndValue(0.0);

    setJointStatus(JOINT_OK);//default?

    numError = 0;
    ui->errorCount->setText("0");

    //want to hide window when fadeout finishes
    connect(errorFadeOut,SIGNAL(finished()),this,SLOT(hideErrorWindow()));
    connect(jointFadeOut,SIGNAL(finished()),this,SLOT(hideJointWindow()));

    window_control_pub_ = n_.advertise<std_msgs::Int8>( "/flor/ocs/window_control", 1, false);

}

void LogSbar::hideJointWindow()
{
    miniJoint->hide();
}
void LogSbar::hideErrorWindow()
{
    miniError->hide();
}

//wrapper to ignore string from signal
void LogSbar::receiveJointData(int status,QString jointName)
{
    setJointStatus(status);
}
void LogSbar::setJointStatus(int status)
{
    switch(status)
    {
    case JOINT_OK:
        ui->jointStatus->setText("OK");
        ui->jointStatus->setStyleSheet("QLabel{color: green; }");
        break;
    case JOINT_WARN:
        ui->jointStatus->setText("WARN");
        ui->jointStatus->setStyleSheet("QLabel{color: yellow; }");
        break;
    case JOINT_ERROR:
        ui->jointStatus->setText("ERROR");
        ui->jointStatus->setStyleSheet("QLabel{color: red; }");
        break;
    }
}

void LogSbar::receiveErrorData(QString time, QString message)
{
    numError++;
    QString s = QString::number(numError);
    ui->errorCount->setText(s);
    ui->errorCount->setStyleSheet("QLabel{color: red; }");
}

//always used to set 0
void LogSbar::setErrorCount(int num)
{
    numError = num;
    QString s = QString::number(numError);
    ui->errorCount->setText(s);
    ui->errorCount->setStyleSheet("QLabel{color: rgb(80,80,80); }");
}

//methods for emitting signals with reference to parent
void LogSbar::notifyMiniError()
{
    Q_EMIT makeErrorActive();
}
void LogSbar::notifyMiniJoint()
{
    Q_EMIT makeJointActive();
}

LogSbar::~LogSbar()
{    
    delete(errorFadeIn);
    delete(errorFadeOut);
    delete(jointFadeIn);
    delete(jointFadeOut);
    delete ui;    
}

ErrorWidget::ErrorWidget(QWidget * parent):
    QWidget(parent)
{
    //grab reference to LogSbar  //tight coupling but only special occurence
    myParent = qobject_cast<LogSbar*>(this->parent()->parent()->parent());
}

//mouse enter/leave trigger these methods
void ErrorWidget::enterEvent(QEvent * event)
{
    //pull up mini error window
    myParent->getMiniError()->show();
    myParent->getErrorFadeIn()->start();
    myParent->getMiniError()->setGeometry(myParent->getUi()->statusBar->mapToGlobal(QPoint(0,0)).x(),myParent->getUi()->statusBar->mapToGlobal(QPoint(0,0)).y() - 180,420, 230);
    myParent->notifyMiniError();
}
void ErrorWidget::leaveEvent(QEvent * event)
{
    myParent->getErrorFadeOut()->start();
    myParent->setErrorCount(0);    
}

void ErrorWidget::mousePressEvent(QMouseEvent * event)
{
    //pull up log window from main view
    std_msgs::Int8 cmd;
    cmd.data = WINDOW_SYSTEM_LOG;
    myParent->getWindowPublisher().publish(cmd);

}

ErrorWidget::~ErrorWidget()
{
}


JointWidget::JointWidget(QWidget * parent):
    QWidget(parent)
{
    myParent = qobject_cast<LogSbar*>(this->parent()->parent()->parent());
}

void JointWidget::enterEvent(QEvent * event)
{
    myParent->getMiniJoint()->show();
    myParent->getJointFadeIn()->start();
    myParent->getMiniJoint()->setGeometry(myParent->getUi()->statusBar->mapToGlobal(QPoint(0,0)).x() + 100,myParent->getUi()->statusBar->mapToGlobal(QPoint(0,0)).y() - 200,400, 200);
    myParent->notifyMiniJoint();
}
void JointWidget::leaveEvent(QEvent * event)
{
    myParent->getJointFadeOut()->start();
}

void JointWidget::mousePressEvent(QMouseEvent * event)
{
    //pull up joint window from main view
    std_msgs::Int8 cmd;
    cmd.data = WINDOW_JOINT_STATUS;
    myParent->getWindowPublisher().publish(cmd);
}

JointWidget::~JointWidget()
{

}

