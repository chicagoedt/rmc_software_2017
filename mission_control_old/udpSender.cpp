#include "udpSender.h"
#include <QDebug>
#include <stdexcept>

UDPSender::UDPSender(QObject* parent) : QObject(parent)
{
    _udpSocket = new QUdpSocket(this);
}

UDPSender::~UDPSender()
{
    // Do not destrou Qt objects here.
    // They are all managed by root parent (Window)
}

bool UDPSender::isConnected(void)
{
    return _udpSocket->state() == QAbstractSocket::ConnectedState;
}

void UDPSender::connect(const QString& host, quint16 port )
{
    if( isConnected() )
        return;

    QObject::connect(_udpSocket, SIGNAL(readyRead()),
                     this, SLOT(readPendingDatagrams()));

    _udpSocket->bind(QHostAddress::Any, port);
    _udpSocket->connectToHost(host, port);

    emit statusUpdate( eOK, QString("UDPSender Connect: ") + host + "  " + QString::number(port));
}

void UDPSender::disconnect(void)
{
    if( isConnected() == false )
        return;

    _udpSocket->disconnectFromHost();

    emit statusUpdate( eOK, QString("UDPSender Disconnect"));

    QObject::disconnect(_udpSocket, SIGNAL(readyRead()),
                        this, SLOT(readPendingDatagrams()));
}

void    UDPSender::send(const QByteArray& buffer)
{
    if( isConnected() == false )
        return;

    emit networkMessageTrace(eOut, QString(buffer.toHex()));

    _udpSocket->write(buffer);
}

void    UDPSender::readPendingDatagrams()
{
    emit statusUpdate( eOK, QString("UDPSender Reader Created"));

    QByteArray   datagram;
    QHostAddress host;
    quint16      port;

    while (_udpSocket->hasPendingDatagrams())
    {
        datagram.resize(_udpSocket->pendingDatagramSize());

        _udpSocket->readDatagram(datagram.data(), datagram.size(), &host, &port );

        if( _udpSocket->peerAddress() != host && _udpSocket->peerPort() != port )
            processDatagram(datagram);

        datagram.clear();
    }

    emit statusUpdate( eOK, QString("UDPSender Reader Finished"));
}

void    UDPSender::processDatagram(const QByteArray& buffer)
{
    emit networkMessageTrace(eIn, QString(buffer.toHex()));
}

void    UDPSender::publishMessage(const QByteArray& buffer)
{
    send(buffer);
}
