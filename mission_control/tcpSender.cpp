#include "tcpSender.h"

TCPSender::TCPSender(QObject* parent) : QObject(parent)
{
    _tcpSocket = new QTcpSocket(this);

    QObject::connect(_tcpSocket, SIGNAL(connected()),    this,      SLOT(connected()));
    QObject::connect(_tcpSocket, SIGNAL(disconnected()), this,      SLOT(disconnected()));
    QObject::connect(_tcpSocket, SIGNAL(bytesWritten(qint64)),this, SLOT(bytesWritten(qint64)));
    QObject::connect(_tcpSocket, SIGNAL(readyRead()),    this,      SLOT(readyRead()));
}

TCPSender::~TCPSender()
{
    // Do not destrou Qt objects here.
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

    _tcpSocket->connectToHost(host, port);
}

void    TCPSender::disconnect(void)
{
    if( isConnected() == false )
        return;

    _tcpSocket->disconnect();
}

void    TCPSender::send(const QByteArray& buffer)
{
    if( isConnected() == false)
        return;

    // We copy this buffer so in case Sigmux did not recive it in some
    // specified time it should make request over TCP and in
    // TCPSender::readyRead() last snapshot should be pushed

    _lock.lock();
    _snapShotData = buffer;
    _lock.unlock();

    emit publishUDPMessage(buffer);

    // This is to debug when testing TCP only
    // In final version comment this out
    // sendSnapshot();
}

void    TCPSender::sendSnapshot()
{
    QByteArray  snapShot;

    _lock.lock();

    snapShot = _snapShotData;

    _lock.unlock();

    const char *bytes = snapShot.constData();
    int bytesWritten  = 0;
    int ret           = 0;

    while (bytesWritten < snapShot.size())
    {
        ret = _tcpSocket->write(bytes + bytesWritten);

        if(ret < 0)
            return;

        bytesWritten += ret;
    }
}

void    TCPSender::publishMessage(const QByteArray& buffer)
{
    send(buffer);
}

void    TCPSender::connected()
{
    qDebug() << "connected...";
}

void    TCPSender::disconnected()
{
    qDebug() << "disconnected...";
}

void    TCPSender::bytesWritten(qint64 bytes)
{
    qDebug() << bytes << " bytes written...";
}

void    TCPSender::readyRead()
{
    qDebug() << "reading...";

    // In this case if you recive message from Sigmax asking for data
    // in case on UDP side did not delivered we will send last snapshot
    // sendSnapshot()
}
