#include "arenawindow.h"
#include "ui_arenawindow.h"
#include <vector>
#include <QLabel>
#include <list>

ArenaWindow::ArenaWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::arenaWindow)
{
    ui->setupUi(this);
    statusNotActive();
}

ArenaWindow::~ArenaWindow()
{
    delete ui;
}

void ArenaWindow::on_rmcMessage(const RMCData& msg)
{
    _points.push_back(msg);
}

// display drawing mode
void ArenaWindow::statusActive()
{
    ui->status->setText("DRAWING");
    ui->status->setStyleSheet("color: red");
}

// display not drawing mode
void ArenaWindow::statusNotActive()
{
    ui->status->setText("NOT DRAWING");
    ui->status->setStyleSheet("color: green");
}

// draw the point(s)
void ArenaWindow::drawPoints()
{
    statusActive();

    for(const RMCData &point : _points)
    {

    }
}

// clears all the point(s)
void ArenaWindow::clearPoints()
{
    _points.clear();

    statusNotActive();
}

void ArenaWindow::drawOrientation(){

}


