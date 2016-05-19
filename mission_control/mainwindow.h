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
#include "arenawindow.h"
#include "../rmcDecode/rmcEnDecoder.h"

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
        void    onRMCMessage(const RMCData& data);

    private slots:
        void deviceConnected(const QString& label);
        void deviceDisconnected(void);
        void deviceUpdate(const InputUpdate& state);

        void networkMessageTrace(const eDirection dir,
                                 const QString& message);

        void statusUpdate(   const eStatus& status,
                             const QString& message);

        void statusUDPUpdate(const eStatus& status,
                             const QString& message);

        void statusTCPUpdate(const eStatus& status,
                             const QString& message);

        void updateLCD();
        void bitsUpdate(const QString& bits);

        void actuatorState( int level );
        void diggingState(bool enabled);
        void dockinggState(bool enabled);
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

        void on_rmcMessage(RMCEnDecoder::TVec msg);

        void logTxData(const QByteArray& msg);

        void on_EStopUpdate(bool enable);

        void on_deviceLock(void);

    private:
        void    logTrace(const eStatus& status,
                         const QString& message);
        void    closeConnectors(void);
        void    openNetworkConnection(void);
        void    closeNetworkConnection(void);
        void    resetLCD();

        void    closeEvent(QCloseEvent* event);

    private:
        Ui::MainWindow*     _ui;
        ArenaWindow*        _arenaWindow;
        QLabel*             _labelHost;
        QLabel*             _labelHostName;
        QLabel*             _labelDevice;
        QLabel*             _labelDeviceName;
        QLabel*             _tcpConnectionStatus;
        QFile*              _logger;
        QFile*              _datalogger;
        QTimer*             _lcdTimer;
        QTime               _lcdTimeValue;
        InputThrottler*     _inputThrottler;
        JoystickConnector*  _joystickConnector;
        UDPSender*          _udpSender;
        TCPSender*          _tcpSender;
        StatsMonitor*       _statsMonitor;
        RMCEnDecoder        _rmcDecoder;
};

#endif // MAINWINDOW_H
