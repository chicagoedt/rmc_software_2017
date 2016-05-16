#include "arenawindow.h"
#include "ui_arenawindow.h"
#include <vector>
#include <QLabel>
#include <list>

arenaWindow::arenaWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::arenaWindow)
{
    ui->setupUi(this);
    statusNotActive();
}

arenaWindow::~arenaWindow()
{
    delete ui;
}

// display drawing mode
void arenaWindow::statusActive(){
    ui->status->setText("DRAWING");
    ui->status->setStyleSheet("color: red");
}

// display not drawing mode
void arenaWindow::statusNotActive(){
    ui->status->setText("NOT DRAWING");
    ui->status->setStyleSheet("color: green");
}

// draw the point(s)
void arenaWindow::drawPoints(){
    statusActive();
    int length = points.back().length; //Simpler way to get length & width & orientation
    int width = points.back().width;
    float angle = points.back().orientation;

    points.back().rep = new QLabel;
    points.back().rep->setText("");

    points.back().rep->move(length,width);
}

// clears all the point(s)
void arenaWindow::clearPoints(){
    for(int i = points.size(); i > 0; i--){
        //delete points
        points.pop_back();
    }
    statusNotActive();
}

void arenaWindow::drawOrientation(){

}

void arenaWindow::getPoints(){
    point newPoint;
    //newPoint x =
    //newPoint y =
    //newPoint orientation =
    points.push_back(newPoint);
}
