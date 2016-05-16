#ifndef TCPSENDER_H
#define TCPSENDER_H

#include <QThread>
#include <QTcpSocket>
#include "commonhdr.h"
#include <QMutex>
#include "pointMapper.h"

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
        void    onNewMessage(char* pBUF,int size);

    private:
        qint64  sendSnapshot();

    private:
       QTcpSocket*   _tcpSocket;
       QMutex        _lock;
       QByteArray    _snapShotData;
       char          _receiveBuffer[4];
};

#endif // TCPSENDER_H
