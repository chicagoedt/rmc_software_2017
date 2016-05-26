#include "tcpSender.h"


int debug = 1;

TCPSender::TCPSender(QObject* parent) : QObject(parent)
{
    _tcpSocket = new QTcpSocket(this);


    QObject::connect(_tcpSocket, SIGNAL(connected()),    this,      SLOT(connected()));
    QObject::connect(_tcpSocket, SIGNAL(disconnected()), this,      SLOT(disconnected()));
    QObject::connect(_tcpSocket, SIGNAL(bytesWritten(qint64)),this, SLOT(bytesWritten(qint64)));
    QObject::connect(_tcpSocket, SIGNAL(readyRead()),    this,      SLOT(readyRead()));
    QObject::connect(_tcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
                     this,       SLOT(error(QAbstractSocket::SocketError)));
}

TCPSender::~TCPSender()
{
    // Do not destroy Qt objects here.
    // They are all managed by root parent (Window)
}

bool    TCPSender::isConnected(void)
{
    return _tcpSocket->state() == QAbstractSocket::ConnectedState;
}

void    TCPSender::connect(const QString& host, quint16 port )
{
    if( isConnected() )
        return;

    _tcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    _tcpSocket->connectToHost(host, port);
     emit statusUpdate( eInfo, QString("TCPSender Connect: ") + host + "  " + QString::number(port));
}

void    TCPSender::disconnect(void)
{
    if( isConnected() == false )
        return;

    _tcpSocket->disconnectFromHost();

    emit statusUpdate( eDisconnected, QString("TCPSender Disconnect"));
}

void    TCPSender::error(QAbstractSocket::SocketError)
{
    emit statusUpdate(eDisconnected, QString("TCPSender ") + _tcpSocket->errorString());
    _tcpSocket->disconnectFromHost();
}

qint64    TCPSender::sendSnapshot()
{
    QByteArray  snapShot;

    _lock.lock();

    snapShot = _snapShotData;

    _lock.unlock();

    qint64 ret = _tcpSocket->write(snapShot);

    if( ret < 0)
    {
        emit statusUpdate(eError, QString("TCPSender Write I/O Error"));
        return -1;
    }

    return ret;
}

void    TCPSender::publishMessage(const QByteArray& buffer)
{
    if( isConnected() == false)
        return;

    // We copy this buffer so in case Sigmux did not recive it in some
    // specified time it should make request over TCP and in
    // TCPSender::readyRead() last snapshot should be pushed

    _lock.lock();
    _snapShotData = buffer;
    _lock.unlock();

    //if( debug == 3)
    //{
      //  debug = 1;
      //  emit publishBackupUDPMessage(buffer);
    //}
    //else
    //{
        // This is to debug when testing TCP only
        // In final version comment this out
        sendSnapshot();
       // debug++;
    //}
}

void    TCPSender::connected()
{
    emit statusUpdate(eConnected, QString("TCPSender Connected"));
}

void    TCPSender::disconnected()
{
    emit statusUpdate(eDisconnected, QString("TCPSender Disconnected"));
}

void    TCPSender::bytesWritten(qint64 bytes)
{
    qDebug() << bytes << " bytes out";
}

void    TCPSender::readyRead()
{
    qDebug() << "ready to read";

    if(_tcpSocket->read(reinterpret_cast<char*>(_receiveBuffer.buffer()), MAX_MSG_SIZE) > 0 ){
        qDebug() << "data " << _receiveBuffer[0] << " "
                 << _receiveBuffer[1] << " "
                 << _receiveBuffer[2] << "\n";

        emit onRMCMessage(_receiveBuffer);

    }
    else{
        //not connected
        return;
    }

    // In this case if you recive message from Sigmax asking for data
    // in case on UDP side did not delivered we will send last snapshot
    // sendSnapshot()
}
