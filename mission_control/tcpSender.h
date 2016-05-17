#ifndef TCPSENDER_H
#define TCPSENDER_H

#include <QThread>
#include <QTcpSocket>
#include <QMutex>
#include "commonhdr.h"
#include "../rmcDecode/rmcEnDecoder.h"

class TCPSender :  public QObject
{
    Q_OBJECT

    public:
        explicit TCPSender(QObject* parent = 0L);
        virtual ~TCPSender();

        bool    isConnected(void);
        void    connect(const QString& host, quint16 port );
        void    disconnect(void);


    public slots:
        void    publishMessage(const QByteArray& buffer);

    private slots:
        void    connected();
        void    disconnected();
        void    bytesWritten(qint64 bytes);
        void    readyRead();
        void    error(QAbstractSocket::SocketError socketError);

    signals:
        void    publishBackupUDPMessage(const QByteArray& buffer);
        void    statusUpdate(const eStatus& status, const QString& message);
        void    networkMessageTrace(const eDirection dir,
                                    const QString& message);
        void    onRMCMessage(RMCEnDecoder::TVec msg);

    private:
        qint64  sendSnapshot();

    private:
       QTcpSocket*          _tcpSocket;
       QMutex               _lock;
       QByteArray           _snapShotData;
       RMCEnDecoder::TVec   _receiveBuffer;
};

#endif // TCPSENDER_H
