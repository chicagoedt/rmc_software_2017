#include "arenawindow.h"
#include "ui_arenawindow.h"
#include <vector>
#include <list>
#include <QPainter>

ArenaWindow::ArenaWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::arenaWindow)
{
    setWindowFlags(Qt::Window);
    ui->setupUi(this);
    statusNotActive();
    //setWindowFlags ( Qt::CustomizeWindowHint | Qt::WindowTitleHint);
}

ArenaWindow::~ArenaWindow()
{
    delete ui;
}

void ArenaWindow::on_rmcMessage(const RMCData& msg)
{
    _points.push_back(msg);
    update();
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

// clears all the point(s)
void ArenaWindow::clearPoints()
{
    _points.clear();

    statusNotActive();
}

void ArenaWindow::drawOrientation(){

}

void ArenaWindow::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    for(const RMCData &point : _points)
    {
        painter.drawPoint(point.posX(), point.posY());
    }
}
