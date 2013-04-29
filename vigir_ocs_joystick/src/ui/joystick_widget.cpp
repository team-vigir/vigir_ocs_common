#include "joystick_widget.h"
#include "ui_joystick_widget.h"
#include "ui/template_loader_widget.h"

JoystickWidget::JoystickWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::JoystickWidget)
{
    ui->setupUi(this);

    joystick = new Joystick();

<<<<<<< HEAD
//    // Make signal/slot connections.
//    connect( ui->yaw_dial, SIGNAL( sliderMoved( int ) ), this, SLOT( yawDialChanged() ) );
//    connect( ui->yaw_dial, SIGNAL( released() ), this, SLOT( yawDialReleased() ) );
//    connect( ui->throttle_slider, SIGNAL( sliderMoved( int ) ), this, SLOT( verticalSliderMoved() ) );
//    connect( ui->throttle_slider, SIGNAL( released() ), this, SLOT( verticalSliderReleased() ) );
    QObject::connect( joystick, SIGNAL( throttleUpdated(unsigned char)), this, SLOT( setProgressBar(unsigned char)) );
=======
    // Make signal/slot connections.
    connect( ui->yaw_dial, SIGNAL( valueChanged( int ) ), this, SLOT( yawDialChanged() ) );
    //connect( ui->yaw_dial, SIGNAL( sliderMoved( int ) ), this, SLOT( yawDialReleased() ) );
    connect( ui->throttle_slider, SIGNAL( valueChanged( int ) ), this, SLOT( throttleSliderMoved() ) );
    //connect( ui->throttle_slider, SIGNAL( valueChanged( int ) ), this, SLOT( throttleSliderMoved() ) );
>>>>>>> origin/joystick_test

    ui->yaw_dial->setRange( -127, 127 );

    ui->yaw_dial->setValue( ( int )( joystick->getRobotSteer() ) );

    ui->yaw_dial->setWrapping(false);

    ui->throttle_slider->setRange( 0, 255 );

    ui->throttle_slider->setValue( ( int )( joystick->getRobotThrottle() ) );

    setProgressBar();
}

JoystickWidget::~JoystickWidget()
{
    delete ui;
}

void JoystickWidget::yawDialChanged()
{
    //std::cout << "Yaw changed";

    if (ui->yaw_dial->value() > 127)
    {
        joystick->setRobotSteer( (signed char)-127 );
    }
    else if (ui->yaw_dial->value() < -127)
    {
        joystick->setRobotSteer( (signed char)127 );
    }
    else
    {
        joystick->setRobotSteer( (signed char)( -1 * (ui->yaw_dial->value() ) ) );
    }
}

void JoystickWidget::yawDialReleased()
{
    if (ui->yaw_dial->value() > 127)
    {
        ui->yaw_dial->setValue( 127 );
        joystick->setRobotSteer( (signed char)127 );
    }
    else if (ui->yaw_dial->value() < -127)
    {
        ui->yaw_dial->setValue( -127 );
        joystick->setRobotSteer( (signed char)-127 );
    }
    else
    {
        joystick->setRobotSteer( (signed char)(ui->yaw_dial->value()) );
    }
}

void JoystickWidget::setProgressBar(unsigned char throttle)
{
    ui->throttle_progress_bar->setValue( (int)throttle / 255 * 100 );
}

void JoystickWidget::throttleSliderMoved()
{
    //std::cout << ui->throttle_slider->value() << " ";

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
