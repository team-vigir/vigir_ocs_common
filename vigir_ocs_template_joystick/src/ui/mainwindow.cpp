#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QLabel>
#include <QSignalMapper>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{       
    ui->setupUi(this);
    controller = new vigir_ocs::Controller();
    mapper = new QSignalMapper(this);
    connect(mapper,SIGNAL(mapped(QString)),this,SLOT(setDirection(QString)));

    //map all directional button to unique strings
    mapper->setMapping(this->ui->UpLButton,"L,Up");
    mapper->setMapping(this->ui->UpRButton,"R,Up");
    mapper->setMapping(this->ui->LeftLButton,"L,Left");
    mapper->setMapping(this->ui->LeftRButton,"R,Left");
    mapper->setMapping(this->ui->RightLButton,"L,Right");
    mapper->setMapping(this->ui->RightRButton,"R,Right");
    mapper->setMapping(this->ui->DownLButton,"L,Down");
    mapper->setMapping(this->ui->DownRButton,"R,Down");

    //connect all buttons for mouse presses
    connect(ui->UpLButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->UpRButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->LeftLButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->LeftRButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->RightLButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->RightRButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->DownLButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->DownRButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->selectTemplate,SIGNAL(pressed()),this,SLOT(selectTemplate()));

    //connect buttons for mouse release(reset labels)
    connect(ui->UpLButton,SIGNAL(released()),this,SLOT(disableLeftLabel()));
    connect(ui->UpRButton,SIGNAL(released()),this,SLOT(disableRightLabel()));
    connect(ui->LeftLButton,SIGNAL(released()),this,SLOT(disableLeftLabel()));
    connect(ui->LeftRButton,SIGNAL(released()),this,SLOT(disableRightLabel()));
    connect(ui->RightLButton,SIGNAL(released()),this,SLOT(disableLeftLabel()));
    connect(ui->RightRButton,SIGNAL(released()),this,SLOT(disableRightLabel()));
    connect(ui->DownLButton,SIGNAL(released()),this,SLOT(disableLeftLabel()));
    connect(ui->DownRButton,SIGNAL(released()),this,SLOT(disableRightLabel()));

    //update combo box based on signal from controller
    connect(controller,SIGNAL(updateTemplateComboBox(int)),this,SLOT(populateTemplateComboBox(int)));
    //selection within combox box
    connect(ui->templateComboBox,SIGNAL(currentIndexChanged(int)),controller,SLOT(changeTemplateID(int)));

    //connect buttons for Left and Right arm modes
    connect(ui->leftArmBtn,SIGNAL(pressed()),controller,SLOT(leftModeToggle()));

    //place graphics on buttons
    QPixmap upArrow("/opt/vigir/catkin_ws/src/vigir_ocs_common/vigir_ocs_template_joystick/src/ui/up.png");
    QPixmap rightArrow("/opt/vigir/catkin_ws/src/vigir_ocs_common/vigir_ocs_template_joystick/src/ui/right.png");
    QPixmap leftArrow("/opt/vigir/catkin_ws/src/vigir_ocs_common/vigir_ocs_template_joystick/src/ui/left.png");
    QPixmap downArrow("/opt/vigir/catkin_ws/src/vigir_ocs_common/vigir_ocs_template_joystick/src/ui/down.png");
    QIcon up(upArrow);
    QIcon right(rightArrow);
    QIcon left(leftArrow);
    QIcon down(downArrow);
    ui->UpRButton->setIcon(up);
    ui->UpRButton->setIconSize(upArrow.rect().size()/2);
    ui->UpLButton->setIcon(up);
    ui->UpLButton->setIconSize(upArrow.rect().size()/2);
    ui->LeftLButton->setIcon(left);
    ui->LeftLButton->setIconSize(leftArrow.rect().size()/2);
    ui->LeftRButton->setIcon(left);
    ui->LeftRButton->setIconSize(leftArrow.rect().size()/2);
    ui->DownLButton->setIcon(down);
    ui->DownLButton->setIconSize(downArrow.rect().size()/2);
    ui->DownRButton->setIcon(down);
    ui->DownRButton->setIconSize(downArrow.rect().size()/2);
    ui->RightLButton->setIcon(right);
    ui->RightLButton->setIconSize(rightArrow.rect().size()/2);
    ui->RightRButton->setIcon(right);
    ui->RightRButton->setIconSize(rightArrow.rect().size()/2);

// ui->widget->setStyleSheet(QString("image:url('/opt/vigir/catkin_ws/src/vigir_ocs_common/vigir_ocs_template_joystick/src/ui/up.png');"));

}

void MainWindow::populateTemplateComboBox(int tempId)
{
    //clear comboBox
    while(ui->templateComboBox->count()>0)
    {
        ui->templateComboBox->removeItem(0);
    }

    std::vector<std::string> templates = controller->getTemplateNames();
    int size = templates.size();
    //add new updated contents from vector
    for(int i =0;i<size;i++)
    {        
        ui->templateComboBox->addItem(QString::fromStdString(templates[i]));
    }
    //make item selected
    ui->templateComboBox->setCurrentIndex(tempId);
}

void MainWindow::selectTemplate()
{
    controller->changeTemplate();
}

//clear text to display nothing when no button is pressed
void MainWindow::disableLeftLabel()
{
    ui->directionL->setText("");
}
void MainWindow::disableRightLabel()
{
    ui->directionR->setText("");
}

//set direction based on unique strings
void MainWindow::setDirection(QString str)
{
    float x = 0;
    float z = 0;
    float rotX = 0;
    float rotY = 0;
    if(str.compare("L,Up") == 0)
    {
        ui->directionL->setText("Up");
        z = .01;
    }
    else if (str.compare("R,Up") == 0)
    {
        ui->directionR->setText("Up");
        rotY = 5;
    }
    else if (str.compare("L,Left") == 0)
    {
        ui->directionL->setText("Left");
        x = -.01;
    }
    else if (str.compare("R,Left") == 0)
    {
        ui->directionR->setText("Left");
        rotX = -5;
    }
    else if (str.compare("L,Right") == 0)
    {
        ui->directionL->setText("Right");
        x = .01;
    }
    else if (str.compare("R,Right") == 0)
    {
        ui->directionR->setText("Right");
        rotX = 5;
    }
    else if (str.compare("L,Down") == 0)
    {
        ui->directionL->setText("Down");
        z = -.01;
    }
    else if (str.compare("R,Down") == 0)
    {
        ui->directionR->setText("Down");
        rotY = -5;
    }
    controller->buildmsg(x,z,rotX,rotY);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    keysPressed.insert((Qt::Key) event->key());

    processKeys();

}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    keysPressed.remove((Qt::Key)event->key());

    if(event->key() == Qt::Key_W ||event->key() == Qt::Key_A || event->key() == Qt::Key_S || event->key() == Qt::Key_D )
        disableLeftLabel();
    else if (event->key() == Qt::Key_I ||event->key() == Qt::Key_J || event->key() == Qt::Key_K || event->key() == Qt::Key_L)
        disableRightLabel();
    //ui->glwidget->keyReleaseEvent(event);

}

//handles multiple/single keys pressed
void MainWindow::processKeys()
{
    float x = 0;
    float z = 0;
    float rotX = 0;
    float rotY = 0;

    if(keysPressed.contains(Qt::Key_W))
    {
        ui->directionL->setText("Up");
        z = .1;
    }
    if(keysPressed.contains(Qt::Key_A))
    {
        ui->directionL->setText("Left");
        x = -0.1;
    }
    if(keysPressed.contains(Qt::Key_S))
    {
        ui->directionL->setText("Down");
        z = -.1;
    }
    if(keysPressed.contains(Qt::Key_D))
    {
        ui->directionL->setText("Right");
        x = .1;
    }
    if(keysPressed.contains(Qt::Key_I))
    {
        ui->directionR->setText("Up");
        rotY = 5;
    }
    if(keysPressed.contains(Qt::Key_J))
    {
        ui->directionR->setText("Left");
        rotX = -5;
    }
    if(keysPressed.contains(Qt::Key_K))
    {
        ui->directionR->setText("Down");
        rotY = -5;
    }
    if(keysPressed.contains(Qt::Key_L))
    {
        ui->directionR->setText("Right");
        rotX = 5;
    }
    controller->buildmsg(x,z,rotX,rotY);
}

MainWindow::~MainWindow()
{
    delete(controller);
    delete(mapper);
    delete ui;

}
