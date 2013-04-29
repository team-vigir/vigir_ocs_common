#include "joystick_widget.h"
#include "ui_joystick_widget.h"
#include "ui/template_loader_widget.h"

JoystickWidget::JoystickWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::JoystickWidget)
{
    ui->setupUi(this);

    joystick = new Joystick();

    // Make signal/slot connections.
    connect( ui->yaw_dial, SIGNAL( valueChanged( int ) ), this, SLOT( yawDialChanged() ) );
    //connect( ui->yaw_dial, SIGNAL( sliderMoved( int ) ), this, SLOT( yawDialReleased() ) );
    connect( ui->throttle_slider, SIGNAL( valueChanged( int ) ), this, SLOT( throttleSliderMoved() ) );
    //connect( ui->throttle_slider, SIGNAL( valueChanged( int ) ), this, SLOT( throttleSliderMoved() ) );
    connect( joystick, SIGNAL( throttleUpdated(unsigned char)), this, SLOT( setProgressBar(unsigned char)) );

    ui->yaw_dial->setRange( -127, 127 );

    ui->yaw_dial->setValue( ( int )( joystick->getRobotSteer() ) );

    ui->yaw_dial->setWrapping(false);

    ui->throttle_slider->setRange( 0, 255 );

    ui->throttle_slider->setValue( ( int )( joystick->getRobotThrottle() ) );

    setProgressBar(0);
}

JoystickWidget::~JoystickWidget()
{
    delete ui;
}

void JoystickWidget::yawDialChanged()
{
    char steer = ui->yaw_dial->value();
    if (abs(joystick->getRobotSteer()-(-ui->yaw_dial->value())) > 50) // do this to prevent big jumps
    {
        steer = -joystick->getRobotSteer();
        ui->yaw_dial->setValue(steer);
    }

    joystick->setRobotSteer( (char)( -steer ) );
}

void JoystickWidget::yawDialReleased()
{
    if (ui->yaw_dial->value() > 127)
    {
        ui->yaw_dial->setValue( 127 );
        joystick->setRobotSteer( (char)127 );
    }
    else if (ui->yaw_dial->value() < -127)
    {
        ui->yaw_dial->setValue( -127 );
        joystick->setRobotSteer( (char)-127 );
    }
    else
    {
        joystick->setRobotSteer( (char)(ui->yaw_dial->value()) );
    }
}

void JoystickWidget::setProgressBar(unsigned char throttle)
{
    std::cout << "joystick " << (unsigned int)throttle << std::endl;
    ui->throttle_progress_bar->setValue( (unsigned int)throttle / 255.0 * 100.0 );
}

void JoystickWidget::throttleSliderMoved()
{
    joystick->setRobotThrottle( (unsigned char)(ui->throttle_slider->value()) );
}

void JoystickWidget::throttleSliderReleased()
{
    joystick->setRobotThrottle((unsigned char)(ui->throttle_slider->value()));
}
