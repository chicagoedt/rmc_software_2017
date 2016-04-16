#include "broadcastudp.h"
#include <QDebug>
#include <stdexcept>

BroadcastUDP::BroadcastUDP(QObject* parent) : QObject(parent)
{
    _udpSocket = new QUdpSocket(this);
}

BroadcastUDP::~BroadcastUDP()
{

}

bool BroadcastUDP::IsConnected(void)
{
    return _udpSocket->state() == QAbstractSocket::ConnectedState;
}

void BroadcastUDP::Connect(const QString& host, quint16 port )
{
    if( IsConnected() )
        return;

    connect(_udpSocket, SIGNAL(readyRead()),
            this, SLOT(ReadPendingDatagrams()));

    _udpSocket->bind(QHostAddress::Any, port);
    _udpSocket->connectToHost(host, port);

    emit StatusUpdate( eOK, QString("BroadcastUDP Connect: ") + host + "  " + QString::number(port));
}

void BroadcastUDP::Disconnect(void)
{
    if( IsConnected() == false )
        return;

    _udpSocket->disconnectFromHost();

    emit StatusUpdate( eOK, QString("BroadcastUDP Disconnect"));

    disconnect(_udpSocket, SIGNAL(readyRead()),
               this, SLOT(ReadPendingDatagrams()));
}

void    BroadcastUDP::Send(const QByteArray& buffer)
{
    if( IsConnected() == false )
        return;

    emit NetworkMessageTrace(eOut, QString(buffer.toHex()));

    _udpSocket->write(buffer);
}

void    BroadcastUDP::ReadPendingDatagrams()
{
    emit StatusUpdate( eOK, QString("BroadcastUDP Reader Created"));

    QByteArray   datagram;
    QHostAddress host;
    quint16      port;

    while (_udpSocket->hasPendingDatagrams())
    {
        datagram.resize(_udpSocket->pendingDatagramSize());

        _udpSocket->readDatagram(datagram.data(), datagram.size(), &host, &port );

        if( _udpSocket->peerAddress() != host && _udpSocket->peerPort() != port )
        {
            ProcessTheDatagram(datagram);
        }

        datagram.clear();
    }

    emit StatusUpdate( eOK, QString("BroadcastUDP Reader Finished"));
}

void    BroadcastUDP::ProcessTheDatagram(const QByteArray& buffer)
{
    emit NetworkMessageTrace(eIn, QString(buffer.toHex()));
}

void    BroadcastUDP::PublishMessage(const QByteArray& buffer)
{
    Send(buffer);
}
