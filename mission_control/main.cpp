#include "mainwindow.h"
#include "arenawindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    ArenaWindow map;

    w.initialize();
    w.show();

    map.show();


    return a.exec();
}
