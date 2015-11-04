#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "videoconnector.h"
#include "joystickconnector.h"
#include "statsMonitor.h"
#include <QDateTime>
#include <QStandardItemModel>
#include <stdexcept>

#define TIME_IN_GAME    10  // Init LCD Timer value

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent), _ui(new Ui::MainWindow),
    _videoConnector(0L), _joystickConnector(0L), _udpBroadcaster(0L)
{
    _ui->setupUi(this);

    _labelHost     = new QLabel("Host: ", this);
    _labelHostName = new QLabel("<b>Disconnected</b>", this);

    _labelDevice     = new QLabel(" Device: ", this);
    _labelDeviceName = new QLabel("<b>Scanning...</b>", this);

    _ui->statusBar->addPermanentWidget(_labelHost);
    _ui->statusBar->addPermanentWidget(_labelHostName);
    _ui->statusBar->addPermanentWidget(_labelDevice);
    _ui->statusBar->addPermanentWidget(_labelDeviceName, 1);

    _ui->lcdRateNumber->display( _ui->horizontalRateSlider->value());

    _ui->pushButtonConnect->setStyleSheet("color: green");
}

MainWindow::~MainWindow()
{
    CloseConnectors();

    if( _logger)
        delete _logger;

    if( _ui )
        delete _ui;
}

void MainWindow::Initialize()
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

    _udpBroadcaster  = new BroadcastUDP(this);

    connect(_udpBroadcaster, SIGNAL(NetworkMessageTrace(const BroadcastUDP::eDirection, const QString&)),
                                      this, SLOT(NetworkMessageTrace(const BroadcastUDP::eDirection, const QString&)));

    _inputThrottler    = new InputThrottler(this);
    _joystickConnector = new JoystickConnector(this);
    _statsMonitor      = new StatsMonitor(this);

    connect(_joystickConnector, SIGNAL(DeviceConnected(const QString&)),
                                       this, SLOT(DeviceConnected(const QString&)));

    connect(_joystickConnector, SIGNAL(DeviceDisconnected(void)),
                                       this, SLOT(DeviceDisconnected(void)));

    connect(_joystickConnector, SIGNAL(StatusUpdate(const eStatus&, const QString&)),
                                       this, SLOT(StatusUpdate(const eStatus&, const QString&)));

    connect(_joystickConnector, SIGNAL(DeviceUpdate(const InputUpdate&)),
                                       this, SLOT(DeviceUpdate(const InputUpdate&)));

    connect( this, SIGNAL(UpdateRateChanged(unsigned int)), _inputThrottler,
                                       SLOT(UpdateRateChanged(unsigned int)));

    connect(_inputThrottler, SIGNAL(StatusUpdate(const eStatus&, const QString&)),
                                       this, SLOT(StatusUpdate(const eStatus&, const QString&)));

    connect(_joystickConnector, SIGNAL(DeviceUpdate(const InputUpdate&)),
                                       _inputThrottler, SLOT(DeviceUpdate(const InputUpdate&)));

    connect(_joystickConnector, SIGNAL(DeviceBtnUpdate(eBtnState, int)),
                                       _inputThrottler, SLOT(DeviceBtnUpdate(eBtnState, int)));

    connect(_joystickConnector, SIGNAL(DeviceBtnUpdate(eBtnState, int)),
                                       this, SLOT(DeviceBtnUpdate(eBtnState, int)));

    connect(_inputThrottler, SIGNAL(PublishMessage(const QByteArray&)),
                                       _udpBroadcaster, SLOT(PublishMessage(const QByteArray&)));

    connect(_inputThrottler, SIGNAL(PublishMessage(const QByteArray&)),
                                       _statsMonitor, SLOT(UpdateTxStats(const QByteArray&)));

    connect(_inputThrottler, SIGNAL(ActuatorState(int)),
                                       this, SLOT(ActuatorState(int)));

    connect(_inputThrottler, SIGNAL(DiggingState(bool)),
                                       this, SLOT(DiggingState(bool)));

    connect(_udpBroadcaster, SIGNAL(StatusUpdate(const eStatus&, const QString&)),
                                       this, SLOT(StatusUpdate(const eStatus&, const QString&)));

    connect(_inputThrottler, SIGNAL(BitsUpdate(const QString&)),
                                       this, SLOT(BitsUpdate(const QString&)));

    connect(_statsMonitor, SIGNAL(StatusUpdate(const eStatus&, const QString&)),
                                       this, SLOT(StatusUpdate(const eStatus&, const QString&)));

    connect(_statsMonitor, SIGNAL(StatsUpdate(const Stats&)),
                                       this, SLOT(StatsUpdate(const Stats&)));

    _inputThrottler->start();
    _statsMonitor->start();
    _joystickConnector->start();

    _lcdTimer     = new QTimer(this);
    connect(_lcdTimer, SIGNAL(timeout()), this, SLOT(updateLCD()));\

    ResetLCD();

    _ui->lcdActuatorNumber->setPalette(QColor::fromRgb(0, 200, 0));
    _ui->labelDig->setStyleSheet("QLabel { background-color : rgb(0, 200, 0) }");
    _ui->labelLock->setStyleSheet("QLabel { background-color : rgb(0, 200, 0) }");

    _ui->labelJoyHz->setText( QString::number(1000.0 / (double)_ui->horizontalRateSlider->value(),
                                              'f', 2) );
}

void MainWindow::DeviceBtnUpdate( eBtnState state, int btnID )
{
    if( state == eDown && btnID == 2 ) // Btn labeled 3 on joy
    {
        if( _joystickConnector->ToggleInputLock() )
        {
            _statsMonitor->ToggleInputLock(true);
            _ui->labelLock->setText("Control Locked");
            _ui->labelLock->setStyleSheet("QLabel { background-color : rgb(0, 200, 0) }");
        }
        else
        {
            _statsMonitor->ToggleInputLock(false);
            _ui->labelLock->setText("Control Enabled");
            _ui->labelLock->setStyleSheet("QLabel { background-color : rgb(250, 0, 0) }");
        }
    }
}

void MainWindow::ResetLCD()
{
    //TIME_IN_GAME
    _ui->countdownTimer->display("10.00");
    _lcdTimeValue = QTime(0, TIME_IN_GAME, 0);
    _ui->countdownTimer->setPalette(QColor::fromRgb(0, 200, 0));
}

void MainWindow::BitsUpdate(const QString& bits)
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

void MainWindow::DeviceConnected(const QString& label)
{
    _labelDeviceName->setText( QString("<b>%1</b>").arg(label));
}

void MainWindow::DeviceDisconnected(void)
{
    _labelDeviceName->setText("<b>Scanning...</b>");
}

void MainWindow::StatusUpdate(const eStatus& status,
                              const QString& message)
{
    LogTrace(status, message);
}

void MainWindow::DeviceUpdate(const InputUpdate& state)
{
    _ui->labelJoyXLeftValue->setText(QString::number( state.AxisLeft().X()));
    _ui->labelJoyYLeftValue->setText(QString::number( state.AxisLeft().Y()));

    _ui->labelJoyXRightValue->setText(QString::number( state.AxisRight().X()));
    _ui->labelJoyYRightValue->setText(QString::number( state.AxisRight().Y()));
}

void MainWindow::ActuatorState( int level )
{
    _ui->lcdActuatorNumber->display( level );
    _ui->progressBarActuator->setValue( level );

    if( level == 0)
        _ui->lcdActuatorNumber->setPalette(QColor::fromRgb(240, 0, 0));
    else if (level == 3)
        _ui->lcdActuatorNumber->setPalette(QColor::fromRgb(255, 255, 0));
    else
        _ui->lcdActuatorNumber->setPalette(QColor::fromRgb(255, 135, 0));
}

void MainWindow::DiggingState(bool enabled)
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

    emit UpdateRateChanged(_ui->horizontalRateSlider->value());
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
            CloseNetworkConnection();
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
    if( _udpBroadcaster->IsConnected() == false )
    {
        _udpBroadcaster->Connect(_ui->hostAddressTextBox->text(), (quint16)_ui->spinBoxControlPort->value() );
        _ui->pushButtonConnect->setStyleSheet("color: red");
        _ui->pushButtonConnect->setText("Disconnect");
        _ui->spinBoxControlPort->setEnabled(false);
        _ui->spinBoxVideoPort->setEnabled(false);
        _ui->hostAddressTextBox->setEnabled(false);
        _labelHostName->setText("<b>Connected</b>");
        _statsMonitor->ToggleConnectionState(true);
    }
}

void MainWindow::CloseNetworkConnection()
{
    if( _udpBroadcaster->IsConnected() )
    {
        _udpBroadcaster->Disconnect();
        _ui->pushButtonConnect->setStyleSheet("color: green");
        _ui->pushButtonConnect->setText("Connect");
        _ui->spinBoxControlPort->setEnabled(true);
        _ui->spinBoxVideoPort->setEnabled(true);
        _ui->hostAddressTextBox->setEnabled(true);
        _labelHostName->setText("<b>Disconnected</b>");
        _statsMonitor->ToggleConnectionState(false);
    }
}

void MainWindow::CloseConnectors(void)
{
     if( _videoConnector )
     {
         _videoConnector->terminate();
         _videoConnector->wait();
         delete _videoConnector;
     }

     if( _joystickConnector )
     {
         _joystickConnector->Quit();
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

     if( _udpBroadcaster )
         _udpBroadcaster->Disconnect();
}

void    MainWindow::LogTrace(const eStatus& status,
                             const QString& message)
{
    QString msg = QDateTime::currentDateTime().toString("hh:mm:ss");

    if( status == eOK )
    {
        msg += " OK  - ";
        msg +=  message;

        qDebug() << message;
    }
    else
    {
        msg += " ERR - ";
        msg +=  message;

        qWarning() << message;
    }

    if( _logger->isOpen() == false )
        return;

    QTextStream textStreamLogger(_logger);

    textStreamLogger << "-- " << message;
}

void    MainWindow::NetworkMessageTrace(const BroadcastUDP::eDirection dir,
                                        const QString& message)
{
    if( _logger->isOpen() == false )
        return;

    QTextStream textStreamLogger(_logger);

    if( dir == BroadcastUDP::eIn )
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
    ResetLCD();

    _ui->startTimeButton->setText("Start Time");
}

void MainWindow::StatsUpdate(const Stats& stats)
{
    _ui->lineEditTxTotalBytes->setText(QString::number(stats.TxTotalBytes()));
    _ui->lineEditTxBytesPerSec->setText(QString::number(stats.TxBytesPerSec()));
    _ui->lineEditTxPacketPerSec->setText(QString::number(stats.TxPacketPerSec()));

    _ui->lineEditRxTotalBytes->setText(QString::number(stats.RxTotalBytes()));
    _ui->lineEditRxBytesPerSec->setText(QString::number(stats.RxBytesPerSec()));
    _ui->lineEditRxPacketPerSec->setText(QString::number(stats.RxPacketPerSec()));
}

void MainWindow::on_pushButtonResetStats_clicked()
{
    _statsMonitor->ResetStats();
}
