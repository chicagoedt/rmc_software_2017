#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "joystickconnector.h"
#include "statsMonitor.h"
#include <QDateTime>
#include <QStandardItemModel>
#include <QCloseEvent>
#include <stdexcept>

#define TIME_IN_GAME    10  // Init LCD Timer value
const QString MainWindow::APP_NAME = QString("Mission Control");

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent), _ui(new Ui::MainWindow),
    _joystickConnector(0L), _udpSender(0L), _tcpSender(0L)
{
    _ui->setupUi(this);

    _labelHost       = new QLabel("Host: ", this);
    _labelHostName   = new QLabel("<b>Disconnected</b>", this);

    _labelDevice     = new QLabel(" Device: ", this);
    _labelDeviceName = new QLabel("<b>Scanning...</b>", this);

    _ui->statusBar->addPermanentWidget(_labelHost);
    _ui->statusBar->addPermanentWidget(_labelHostName);
    _ui->statusBar->addPermanentWidget(_labelDevice);
    _ui->statusBar->addPermanentWidget(_labelDeviceName, 1);

    _streamTCP = _ui->tcpStreamCheckBox->isChecked(); // Whether or not to stream joystick data over TCP or not (UDP)

    _ui->lcdRateNumber->display( _ui->horizontalRateSlider->value());

    _ui->pushButtonConnect->setStyleSheet("color: green");

    _ui->tcpConnectionStatus->setText("TCP NOT CONNECTED");
    _ui->tcpConnectionStatus->setStyleSheet("color: green");
}

MainWindow::~MainWindow()
{
    closeConnectors();

    if( _logger)
        delete _logger;

    if( _ui )
        delete _ui;
}

void MainWindow::initialize()
{
    if( _joystickConnector )
        return;

    qRegisterMetaType<eStatus>("eStatus");
    qRegisterMetaType<eStatus>("eStatus");
    qRegisterMetaType<InputUpdate>("InputUpdate");
    qRegisterMetaType<eBtnState>("eBtnState");
    qRegisterMetaType<Stats>("Stats");

    _logger = new QFile("EDTPanel.log");
    _ui->pushButtonLog->setStyleSheet("color: green");

    _udpSender  = new UDPSender(this);
    _tcpSender  = new TCPSender(this);


    connect(_udpSender, SIGNAL(networkMessageTrace(const eDirection, const QString&)),
                  this, SLOT(networkMessageTrace(const eDirection, const QString&)));

    connect(_tcpSender, SIGNAL(networkMessageTrace(const eDirection, const QString&)),
                  this, SLOT(networkMessageTrace(const eDirection, const QString&)));

    _inputThrottler    = new InputThrottler(this);
    _joystickConnector = new JoystickConnector(this);
    _statsMonitor      = new StatsMonitor(this);

    connect(_joystickConnector, SIGNAL(deviceConnected(const QString&)),
                                       this, SLOT(deviceConnected(const QString&)));

    connect(_joystickConnector, SIGNAL(deviceDisconnected(void)),
                                       this, SLOT(deviceDisconnected(void)));

    connect(_joystickConnector, SIGNAL(statusUpdate(const eStatus&, const QString&)),
                                       this, SLOT(statusUpdate(const eStatus&, const QString&)));

    connect(_joystickConnector, SIGNAL(deviceUpdate(const InputUpdate&)),
                                       this, SLOT(deviceUpdate(const InputUpdate&)));

    connect( this, SIGNAL(updateRateChanged(unsigned int)), _inputThrottler,
                                       SLOT(UpdateRateChanged(unsigned int)));

    connect(_inputThrottler, SIGNAL(StatusUpdate(const eStatus&, const QString&)),
                                       this, SLOT(statusUpdate(const eStatus&, const QString&)));

    connect(_joystickConnector, SIGNAL(deviceUpdate(const InputUpdate&)),
                                       _inputThrottler, SLOT(DeviceUpdate(const InputUpdate&)));


    connect(_joystickConnector, SIGNAL(deviceBtnUpdate(eBtnState, int)),
                                       _inputThrottler, SLOT(DeviceBtnUpdate(eBtnState, int)));

    connect(_joystickConnector, SIGNAL(deviceBtnUpdate(eBtnState, int)),
                                       this, SLOT(deviceBtnUpdate(eBtnState, int)));

    //if(_streamTCP)
    //{
        connect(_inputThrottler, SIGNAL(PublishMessage(const QByteArray&)),
                                           _tcpSender, SLOT(publishMessage(const QByteArray&)));
    //}
    //else
    //{
    //    connect(_inputThrottler, SIGNAL(PublishMessage(const QByteArray&)),
    //                                       _udpSender, SLOT(publishMessage(const QByteArray&)));
    //}


    connect(_inputThrottler, SIGNAL(PublishMessage(const QByteArray&)),
                                       _statsMonitor, SLOT(updateTxStats(const QByteArray&)));

    connect(_inputThrottler, SIGNAL(ActuatorState(int)),
                                       this, SLOT(actuatorState(int)));

    connect(_inputThrottler, SIGNAL(DiggingState(bool)),
                                       this, SLOT(diggingState(bool)));

    connect(_tcpSender, SIGNAL(publishBackupUDPMessage(const QByteArray&)),
                                       _udpSender, SLOT(publishMessage(const QByteArray&)));

    connect(_udpSender, SIGNAL(statusUpdate(const eStatus&, const QString&)),
                                       this, SLOT(statusUDPUpdate(const eStatus&, const QString&)));

    connect(_tcpSender, SIGNAL(statusUpdate(const eStatus&, const QString&)),
                                       this, SLOT(statusTCPUpdate(const eStatus&, const QString&)));

    connect(_inputThrottler, SIGNAL(BitsUpdate(const QString&)),
                                       this, SLOT(bitsUpdate(const QString&)));

    connect(_statsMonitor, SIGNAL(statusUpdate(const eStatus&, const QString&)),
                                       this, SLOT(statusUpdate(const eStatus&, const QString&)));

    connect(_statsMonitor, SIGNAL(statsUpdate(const Stats&)),
                                       this, SLOT(statsUpdate(const Stats&)));

    connect(_tcpSender, SIGNAL(onRMCMessage(RMCEnDecoder::TVec)),
                                       this, SLOT(on_rmcMessage(RMCEnDecoder::TVec msg)));

    _inputThrottler->start();
    _statsMonitor->start();
    _joystickConnector->start();

    _lcdTimer     = new QTimer(this);
    connect(_lcdTimer, SIGNAL(timeout()), this, SLOT(updateLCD()));\

    resetLCD();

    _ui->lcdActuatorNumber->setPalette(QColor::fromRgb(0, 200, 0));
    _ui->labelDig->setStyleSheet("QLabel { background-color : rgb(0, 200, 0) }");
    _ui->labelLock->setStyleSheet("QLabel { background-color : rgb(0, 200, 0) }");

    _ui->labelJoyHz->setText( QString::number(1000.0 / (double)_ui->horizontalRateSlider->value(),
                                              'f', 2) );
}

void    MainWindow::closeEvent(QCloseEvent* event)
{
    QMessageBox::StandardButton resBtn = QMessageBox::question( this, APP_NAME,
                                                                tr("Are you sure?\n"),
                                                                QMessageBox::No | QMessageBox::Yes,
                                                                QMessageBox::No);
    if (resBtn != QMessageBox::Yes)
        event->ignore();
    else
        event->accept();
}

void MainWindow::deviceBtnUpdate( eBtnState state, int btnID )
{
    if( state == eDown && btnID == 0 ) // Btn labeled 1 on joy
    {
        if( _joystickConnector->toggleInputLock() )
        {
            _statsMonitor->toggleInputLock(true);
            _ui->labelLock->setText("Control Locked");
            _ui->labelLock->setStyleSheet("QLabel { background-color : rgb(0, 200, 0) }");
        }
        else
        {
            _statsMonitor->toggleInputLock(false);
            _ui->labelLock->setText("Control Enabled");
            _ui->labelLock->setStyleSheet("QLabel { background-color : rgb(250, 0, 0) }");
        }
    }
}

void MainWindow::resetLCD()
{
    //TIME_IN_GAME
    _ui->countdownTimer->display("10.00");
    _lcdTimeValue = QTime(0, TIME_IN_GAME, 0);
    _ui->countdownTimer->setPalette(QColor::fromRgb(0, 200, 0));
}

void MainWindow::bitsUpdate(const QString& bits)
{
    _ui->labelBits->setText(bits);
}

void MainWindow::updateLCD()
{
    _lcdTimeValue.setHMS(0, _lcdTimeValue.addSecs(-1).minute(),
                            _lcdTimeValue.addSecs(-1).second());

    int sec = QTime(0,0).secsTo(_lcdTimeValue);

    if( sec == 60 )
        _ui->countdownTimer->setPalette(QColor::fromRgb(240, 0, 0));
    else if(sec == 0)
    {
        _lcdTimer->stop();
        _ui->resetTimeButton->setEnabled(true);
        _ui->startTimeButton->setEnabled(false);
    }

    _ui->countdownTimer->display(_lcdTimeValue.toString());
}

void MainWindow::deviceConnected(const QString& label)
{
    _labelDeviceName->setText( QString("<b>%1</b>").arg(label));
}

void MainWindow::deviceDisconnected(void)
{
    _labelDeviceName->setText("<b>Scanning...</b>");
}

void MainWindow::statusUpdate(const eStatus& status,
                              const QString& message)
{
    logTrace(status, message);
}

void MainWindow::statusUDPUpdate(const eStatus& status,
                                 const QString& message)
{
    logTrace(status, message);
}

void MainWindow::statusTCPUpdate(const eStatus& status,
                                 const QString& message)
{
    logTrace(status, message);
}

void MainWindow::deviceUpdate(const InputUpdate& state)
{
    _ui->labelJoyXLeftValue->setText(QString::number( state.axisLeft().X()));
    _ui->labelJoyYLeftValue->setText(QString::number( state.axisLeft().Y()));

    _ui->labelJoyXRightValue->setText(QString::number( state.axisRight().X()));
    _ui->labelJoyYRightValue->setText(QString::number( state.axisRight().Y()));
}

void MainWindow::actuatorState( int level )
{
    _ui->lcdActuatorNumber->display( level );
    _ui->progressBarActuator->setValue( level );

    if( level == 0)
        _ui->lcdActuatorNumber->setPalette(QColor::fromRgb(240, 0, 0));
    else if (level == 2)
        _ui->lcdActuatorNumber->setPalette(QColor::fromRgb(255, 255, 0));
    else
        _ui->lcdActuatorNumber->setPalette(QColor::fromRgb(255, 135, 0));
}

void MainWindow::diggingState(bool enabled)
{
    if( enabled )
    {
        _ui->labelDig->setText("ON");
        _ui->labelDig->setStyleSheet("QLabel { background-color : red; }");
    }
    else
    {
        _ui->labelDig->setText("OFF");
        _ui->labelDig->setStyleSheet("QLabel { background-color : rgb(0, 200, 0) }");
    }
}

void MainWindow::on_horizontalRateSlider_sliderReleased()
{
    _ui->lcdRateNumber->display( _ui->horizontalRateSlider->value());

    emit updateRateChanged(_ui->horizontalRateSlider->value());
}

void MainWindow::on_horizontalRateSlider_valueChanged(int value)
{
    _ui->lcdRateNumber->display(value);

    _ui->labelJoyHz->setText( QString::number(1000.0 / (double)value, 'f', 2) );
}

void MainWindow::on_pushButtonConnect_clicked()
{
    try
    {
        if( _ui->pushButtonConnect->isChecked() )
            OpenNetworkConnection();
        else
            closeNetworkConnection();
    }
    catch( std::runtime_error& err)
    {
        QMessageBox msgBox;
        msgBox.setText(err.what());
        msgBox.setInformativeText("ERROR!!!");
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.exec();
        return;
    }
}

void MainWindow::OpenNetworkConnection()
{
    if( _udpSender->isConnected() == false )
    {
        _udpSender->connect(_ui->hostAddressTextBox->text(), (quint16)_ui->spinBoxUDPPort->value() );
        _ui->pushButtonConnect->setStyleSheet("color: red");
        _ui->pushButtonConnect->setText("Disconnect");
        _ui->spinBoxUDPPort->setEnabled(false);
        _ui->spinBoxTCPPort->setEnabled(false);
        _ui->hostAddressTextBox->setEnabled(false);
        _labelHostName->setText("<b>Connected</b>");
        _statsMonitor->toggleConnectionState(true);
    }

    if( _tcpSender->isConnected() == false )
    {
        _tcpSender->connect(_ui->hostAddressTextBox->text(), (quint16)_ui->spinBoxTCPPort->value() );
       // _ui->tcpConnectionStatus->setText("TCP CONNECTED");
        //_ui->tcpConnectionStatus->setStyleSheet("color: red");
        _ui->pushButtonConnect->setStyleSheet("color: red");
        _ui->pushButtonConnect->setText("Disconnect");
        _ui->spinBoxUDPPort->setEnabled(false);
        _ui->spinBoxTCPPort->setEnabled(false);
        _ui->hostAddressTextBox->setEnabled(false);
        _labelHostName->setText("<b>Connected</b>");
        _statsMonitor->toggleConnectionState(true);
    }
}

void MainWindow::closeNetworkConnection()
{
    if( _udpSender->isConnected() )
    {
        _udpSender->disconnect();
        _ui->pushButtonConnect->setStyleSheet("color: green");
        _ui->pushButtonConnect->setText("Connect");
        _ui->spinBoxUDPPort->setEnabled(true);
        _ui->spinBoxTCPPort->setEnabled(true);
        _ui->hostAddressTextBox->setEnabled(true);
        _labelHostName->setText("<b>Disconnected</b>");
        _statsMonitor->toggleConnectionState(false);
    }

    if( _tcpSender->isConnected() )
    {
        _tcpSender->disconnect();
        _ui->pushButtonConnect->setStyleSheet("color: green");
        _ui->pushButtonConnect->setText("Connect");
        _ui->spinBoxUDPPort->setEnabled(true);
        _ui->spinBoxTCPPort->setEnabled(true);
        _ui->hostAddressTextBox->setEnabled(true);
        _labelHostName->setText("<b>Disconnected</b>");
        _statsMonitor->toggleConnectionState(false);
    }
}

void MainWindow::closeConnectors(void)
{
     if( _joystickConnector )
     {
         _joystickConnector->quit();
         delete _joystickConnector;
     }

     if( _inputThrottler )
     {
         _inputThrottler->terminate();
         _inputThrottler->wait();
         delete _inputThrottler;
     }

     if( _statsMonitor )
     {
         _statsMonitor->terminate();
         _statsMonitor->wait();
         delete _statsMonitor;
     }

     if( _tcpSender )
         _tcpSender->disconnect();

     if( _udpSender )
         _udpSender->disconnect();
}

void    MainWindow::logTrace(const eStatus& status,
                             const QString& message)
{
    QString msg = QDateTime::currentDateTime().toString("hh:mm:ss");

    switch(status)
    {
        case eInfo:
            msg += " INF  - ";
            msg +=  message;

            qDebug() << message;
            break;

        case eConnected:
            msg += " CON - ";
            msg +=  message;

            qWarning() << message;
            break;

        case eDisconnected:
            msg += " DIS - ";
            msg +=  message;

            qWarning() << message;
            break;

        case eError:
            msg += " ERR - ";
            msg +=  message;

            qWarning() << message;
            break;
    }

    if( _logger->isOpen() == false )
        return;

    QTextStream textStreamLogger(_logger);

    textStreamLogger << "-- " << message;
}

void    MainWindow::networkMessageTrace(const eDirection dir,
                                        const QString& message)
{
    if( _logger->isOpen() == false )
        return;

    QTextStream textStreamLogger(_logger);

    if( dir == eIn )
        textStreamLogger << "<- " << message;
    else
        textStreamLogger << "-> " << message;
}

void MainWindow::on_pushButtonLog_clicked()
{
    if( _ui->pushButtonLog->isChecked() )
    {
        _ui->pushButtonLog->setStyleSheet("color: red");
        _ui->pushButtonLog->setText("Log : On");
        _logger->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append);

         QTextStream textStreamLogger(_logger);
         textStreamLogger << "++++ Opened ++++\n";
    }
    else
    {
        QTextStream textStreamLogger(_logger);
        textStreamLogger << "---- Closed ----\n";
        _logger->close();
        _ui->pushButtonLog->setText("Log : Off");
        _ui->pushButtonLog->setStyleSheet("color: green");
    }
}

void MainWindow::on_radioButtonSafe_clicked()
{
    _inputThrottler->SetMode(InputThrottler::eSafety);
}

void MainWindow::on_radioButtonAuto_clicked()
{
    _inputThrottler->SetMode(InputThrottler::eAuto);
}

void MainWindow::on_radioButtonMan_clicked()
{
    _inputThrottler->SetMode(InputThrottler::eManual);
}

void MainWindow::on_startTimeButton_clicked()
{
    if( _lcdTimer->isActive())
    {
        _lcdTimer->stop();
        _ui->resetTimeButton->setEnabled(true);
        _ui->startTimeButton->setText("Restart Time");
    }
    else
    {
        _ui->resetTimeButton->setEnabled(false);
        _ui->countdownTimer->setPalette(Qt::green);
        _lcdTimer->start(1000);
        _ui->startTimeButton->setText("Pause Time");
    }
}

void MainWindow::on_resetTimeButton_clicked()
{
    resetLCD();

    _ui->startTimeButton->setText("Start Time");
}

void MainWindow::statsUpdate(const Stats& stats)
{
    _ui->lineEditTxTotalBytes->setText(QString::number(stats.txTotalBytes()));
    _ui->lineEditTxBytesPerSec->setText(QString::number(stats.txBytesPerSec()));
    _ui->lineEditTxPacketPerSec->setText(QString::number(stats.txPacketPerSec()));

    _ui->lineEditRxTotalBytes->setText(QString::number(stats.rxTotalBytes()));
    _ui->lineEditRxBytesPerSec->setText(QString::number(stats.rxBytesPerSec()));
    _ui->lineEditRxPacketPerSec->setText(QString::number(stats.rxPacketPerSec()));
}

void MainWindow::on_pushButtonResetStats_clicked()
{
    _statsMonitor->resetStats();
}

void MainWindow::on_tcpStreamCheckBox_clicked()
{

}

void MainWindow::on_rmcMessage(RMCEnDecoder::TVec msg)
{
    const RMCData& rmcData = _rmcDecoder.decodeMessage(msg);

    //_ui->tcpConnectionStatus->setText("TCP CONNECTED");
    //_ui->tcpConnectionStatus->setStyleSheet("color: red");
}
