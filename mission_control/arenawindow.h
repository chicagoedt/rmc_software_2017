#ifndef ARENAWINDOW_H
#define ARENAWINDOW_H

#include <QWidget>
#include <QLabel>
#include <list>
#include "../rmcDecode/rmcData.h"

namespace Ui {
class arenaWindow;
}

class ArenaWindow : public QWidget
{
    Q_OBJECT

    public:

        explicit ArenaWindow(QWidget *parent = 0);
                ~ArenaWindow();

        void statusActive();
        void statusNotActive();

        void drawPoints();
        void clearPoints();
        void drawOrientation();

    private slots:
        void on_rmcMessage(const RMCData& msg);

    private:
        Ui::arenaWindow     *ui;
        QLabel              *status;
        std::list<RMCData>  _points;
};

#endif // ARENAWINDOW_H

// 3 bytes, 24 bits total
// first 5 bits - orientation
// next 10 bits - length
// last 9 bits - width
