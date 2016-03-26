#ifndef TCPSENDER_H
#define TCPSENDER_H

#include <QThread>
#include <QTcpSocket>
#include "commonhdr.h"

class TCPSender :  public QObject
{
    Q_OBJECT

    public:
        explicit TCPSender(QObject* parent = 0L);
        virtual ~TCPSender();

        bool    isConnected(void);
        void    connect(const QString& host, quint16 port );
        void    disconnect(void);
        void    send(const QByteArray& buffer);

    public slots:
        void    publishMessage(const QByteArray& buffer);
        void    connected();
        void    disconnected();
        void    bytesWritten(qint64 bytes);
        void    readyRead();

    signals:
        void    publishUDPMessage(const QByteArray& buffer);

    private:
        void    sendSnapshot();

    private:
       QTcpSocket*   _tcpSocket;
       QMutex        _lock;
       QByteArray    _snapShotData;
};

#endif // TCPSENDER_H
