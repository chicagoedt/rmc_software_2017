#ifndef BROADCASTUDP_H
#define BROADCASTUDP_H

#include <QThread>
#include <QUdpSocket>
#include "commonhdr.h"

class BroadcastUDP : public QObject
{
        Q_OBJECT

        public:
            enum eDirection
            {
                eOut,
                eIn
            };

           explicit BroadcastUDP(QObject* parent = 0L);
           virtual ~BroadcastUDP();

           bool     IsConnected(void);
           void     Connect(const QString& host, quint16 port );
           void     Disconnect(void);
           void     Send(const QByteArray& buffer);

        public slots:
           void     ReadPendingDatagrams(void);
           void     PublishMessage(const QByteArray& buffer);

        signals:
           void    NetworkMessageTrace(const BroadcastUDP::eDirection dir,
                                       const QString& message);
           void    StatusUpdate(const eStatus& status, const QString& message);

        private:
           void    ProcessTheDatagram(const QByteArray& buffer);

        private:
           QUdpSocket*   _udpSocket;
};

#endif // BROADCASTUDP_H
