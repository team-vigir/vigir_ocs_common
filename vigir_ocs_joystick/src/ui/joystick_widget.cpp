#include "joystick_widget.h"
#include "ui_joystick_widget.h"
#include "ui/template_loader_widget.h"

JoystickWidget::JoystickWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::JoystickWidget)
{
    ui->setupUi(this);

//    joystick = new Joystick();

//    // Make signal/slot connections.
//    connect( ui->yaw_dial, SIGNAL( sliderMoved( int ) ), this, SLOT( yawDialChanged() ) );
//    connect( ui->yaw_dial, SIGNAL( released() ), this, SLOT( yawDialReleased() ) );
//    connect( ui->throttle_slider, SIGNAL( sliderMoved( int ) ), this, SLOT( verticalSliderMoved() ) );
//    connect( ui->throttle_slider, SIGNAL( released() ), this, SLOT( verticalSliderReleased() ) );

//    ui->yaw_dial->setRange( -127, 127 );

//    ui->yaw_dial->setValue( ( int )( joystick->getRobotSteer() ) );

//    ui->throttle_slider->setRange( 0, 255 );

//    ui->throttle_slider->setValue( ( int )( joystick->getRobotThrottle() ) );

//    setProgressBar();
}

JoystickWidget::~JoystickWidget()
{
    delete ui;
}

void JoystickWidget::yawDialChanged()
{
    if (ui->yaw_dial->value() > 127)
    {
        joystick->setRobotSteer( (signed char)127 );
    }
    else if (ui->yaw_dial->value() < -127)
    {
        joystick->setRobotSteer( (signed char)-127 );
    }
    else
    {
        joystick->setRobotSteer( (signed char)(ui->yaw_dial->value()) );
    }
}

void JoystickWidget::yawDialReleased()
{
    if (ui->yaw_dial->value() > 127)
    {
        joystick->setRobotSteer( (signed char)127 );
    }
    else if (ui->yaw_dial->value() < -127)
    {
        joystick->setRobotSteer( (signed char)-127 );
    }
    else
    {
        joystick->setRobotSteer( (signed char)(ui->yaw_dial->value()) );
    }
}

void JoystickWidget::setProgressBar()
{
    unsigned char throttle = joystick->getRobotThrottle();
    ui->throttle_progress_bar->setValue( (int)throttle / 255 * 100 );
}

void JoystickWidget::throttleSliderMoved()
{
    if (ui->throttle_slider->value() > 255)
    {
        joystick->setRobotThrottle( (unsigned char)255 );
    }
    else if (ui->throttle_slider->value() < 0)
    {
        joystick->setRobotThrottle( (unsigned char)0 );
    }
    else
    {
        joystick->setRobotThrottle( (unsigned char)(ui->throttle_slider->value()) );
    }
}

void JoystickWidget::throttleSliderReleased()
{
    joystick->setRobotThrottle((unsigned char)(ui->throttle_slider->value()));
}
