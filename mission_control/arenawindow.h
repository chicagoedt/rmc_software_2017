#ifndef ARENAWINDOW_H
#define ARENAWINDOW_H

#include <QWidget>
#include <QLabel>
#include <vector>
#include <list>

namespace Ui {
class arenaWindow;
}

class arenaWindow : public QWidget
{
    Q_OBJECT


public:

    explicit arenaWindow(QWidget *parent = 0);
    ~arenaWindow();

    void statusActive();
    void statusNotActive();

    void getPoints();
    void drawPoints();
    void clearPoints();
    void drawOrientation();

    struct point{
        int length;
        int width;
        float orientation;
        QLabel *rep;
    };

    //std::vector<point> points;
    std::list<point> points;
private:
    Ui::arenaWindow *ui;
    QLabel *status;
};

#endif // ARENAWINDOW_H

// 3 bytes, 24 bits total
// first 5 bits - orientation
// next 10 bits - length
// last 9 bits - width
