#include "robotStatus.h"
#include <QVBoxLayout>

robotStatus::robotStatus(QWidget *parent) :
    QWidget(parent)
{
    this->setMinimumSize(300,400);
    //messages = new std::vector<QTableWidgetItem>;

    //bold = new QFont();
    bold.setBold(true);
    //normal = new QFont();
    normal.setBold(false);

    msgTable = new QTableWidget();
    msgTable->setColumnCount(2);
    clearButton = new QPushButton();
    clearButton->setText("Clear Table Contents");
    QVBoxLayout* main_layout = new QVBoxLayout;
    main_layout->addWidget(msgTable);
    main_layout->addWidget(clearButton);
    //std::cout << "Adding layout..." << std::endl;
    setLayout(main_layout);
    msgTable->setColumnWidth(0,50);
    //labels = new QStringList();
    labels.push_back("Time");
    labels.push_back("Message Contents");
    msgTable->setHorizontalHeaderLabels(labels);
    unreadMsgs=0;
}

void robotStatus::on_clearButton_clicked()
{
    msgTable = new QTableWidget();
    msgTable->setColumnCount(2);
    msgTable->setColumnWidth(0,50);
    msgTable->setHorizontalHeaderLabels(labels);
    messages.clear();
    unreadMsgs=0;
}

void robotStatus::on_msgTable_cellClicked(int row, int column)
{
    messages[row-1].setFont(normal);
}

int robotStatus::getNumUnread()
{
    return unreadMsgs;
}

robotStatus::~robotStatus()
{
    //delete ui;
}
