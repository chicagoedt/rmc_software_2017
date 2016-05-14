#ifndef BROADCASTUDP_H
#define BROADCASTUDP_H

#include <QThread>
#include <QUdpSocket>
#include "commonhdr.h"

class UDPSender : public QObject
{
        Q_OBJECT

        public:
            enum eDirection
            {
                eOut,
                eIn
            };

           explicit UDPSender(QObject* parent = 0L);
           virtual ~UDPSender();

           bool     isConnected(void);
           void     connect(const QString& host, quint16 port );
           void     disconnect(void);
           void     send(const QByteArray& buffer);

        public slots:
           void     readPendingDatagrams(void);
           void     publishMessage(const QByteArray& buffer);

        signals:
           void     networkMessageTrace(const UDPSender::eDirection dir,
                                       const QString& message);
           void     statusUpdate(const eStatus& status, const QString& message);

        private:
           void     processDatagram(const QByteArray& buffer);

        private:
           QUdpSocket*   _udpSocket;
};

#endif // BROADCASTUDP_H
