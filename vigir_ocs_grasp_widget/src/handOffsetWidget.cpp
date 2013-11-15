#include "handOffsetWidget.h"
#include "ui_handOffsetWidget.h"

handOffsetWidget::handOffsetWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::handOffsetWidget)
{
    ui->setupUi(this);
}

handOffsetWidget::~handOffsetWidget()
{
    delete ui;
}

void handOffsetWidget::on_roll_inc_clicked()
{
    roll = ui->roll_current->value();
    roll += ui->roll_adj->;
    ui->roll_current->display(roll);
}

void handOffsetWidget::on_roll_dec_clicked()
{
    roll = ui->roll_current->value();
    roll -= ui->roll_adj->value();
    ui->roll_current->display(roll);
}
