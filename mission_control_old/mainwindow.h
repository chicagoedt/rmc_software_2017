#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QFile>
#include <QTimer>
#include <QTime>
#include <QMessageBox>
#include "joystickconnector.h"
#include "inputThrottler.h"
#include "udpSender.h"
#include "tcpSender.h"
#include "statsMonitor.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
        Q_OBJECT

        static const QString APP_NAME;

    public:
        explicit MainWindow(QWidget *parent = 0);
        virtual ~MainWindow();

        void    initialize();

    signals:
        void    updateRateChanged(unsigned int ms) ;

    private slots:
        void deviceConnected(const QString& label);
        void deviceDisconnected(void);
        void deviceUpdate(const InputUpdate& state);

        void networkMessageTrace(const UDPSender::eDirection dir,
                                 const QString& message);

        void statusUpdate(const eStatus& status,
                          const QString& message);

        void updateLCD();
        void bitsUpdate(const QString& bits);

        void actuatorState( int level );
        void diggingState(bool enabled);
        void deviceBtnUpdate( eBtnState state, int btnID );

        void on_pushButtonConnect_clicked();
        void on_horizontalRateSlider_sliderReleased();
        void on_horizontalRateSlider_valueChanged(int value);
        void statsUpdate(const Stats& stats);

        void on_pushButtonLog_clicked();

        void on_radioButtonSafe_clicked();

        void on_radioButtonAuto_clicked();

        void on_radioButtonMan_clicked();

        void on_startTimeButton_clicked();

        void on_resetTimeButton_clicked();

        void on_pushButtonResetStats_clicked();

        void on_tcpStreamCheckBox_clicked();

private:
        void    logTrace(const eStatus& status,
                         const QString& message);
        void    closeConnectors(void);
        void    OpenNetworkConnection(void);
        void    closeNetworkConnection(void);
        void    resetLCD();

        void    closeEvent(QCloseEvent* event);

    private:
        Ui::MainWindow*     _ui;
        QLabel*             _labelHost;
        QLabel*             _labelHostName;
        QLabel*             _labelDevice;
        QLabel*             _labelDeviceName;
        QFile*              _logger;
        QTimer*             _lcdTimer;
        QTime               _lcdTimeValue;
        QTextStream*        _textStreamLogger;
        InputThrottler*     _inputThrottler;
        JoystickConnector*  _joystickConnector;
        UDPSender*          _udpSender;
        TCPSender*          _tcpSender;
        StatsMonitor*       _statsMonitor;
        bool                _streamTCP;
};

#endif // MAINWINDOW_H
