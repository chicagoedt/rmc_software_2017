#ifndef STATSMONITOR_H
#define STATSMONITOR_H

#include "commonhdr.h"
#include <QThread>
#include <QMutex>

class Stats
{
    friend class StatsMonitor;

    public:
        Stats()
        {
           reset();
        }

        inline uint    txTotalBytes()   const { return _txTotalBytes; }
        inline uint    txBytesPerSec()  const { return _txBytesPerSec; }
        inline uint    txPacketPerSec() const { return _txPacketPerSec; }

        inline uint    rxTotalBytes()   const { return _rxTotalBytes; }
        inline uint    rxBytesPerSec()  const { return _rxBytesPerSec; }
        inline uint    rxPacketPerSec() const { return _rxPacketPerSec; }

    private:
        inline void    reset()
        {
            _txTotalBytes       = 0;
            _txBytesPerSec      = 0;
            _txPacketPerSec     = 0;

            _rxTotalBytes       = 0;
            _rxBytesPerSec      = 0;
            _rxPacketPerSec     = 0;
        }

    private:
        uint    _txTotalBytes;
        uint    _txBytesPerSec;
        uint    _txPacketPerSec;

        uint    _rxTotalBytes;
        uint    _rxBytesPerSec;
        uint    _rxPacketPerSec;
};

class StatsMonitor : public QThread
{
    Q_OBJECT
    public:
        StatsMonitor(QObject* parent = 0L);

        void    resetStats();
        void    toggleInputLock(bool state);
        void    toggleConnectionState(bool state);

    signals:
        void    statusUpdate(const eStatus& status, const QString& message);
        void    statsUpdate(const Stats& stats);

    public slots:
        void    updateTxStats(const QByteArray& buffer);

    protected:
       virtual void    run(void);// Q_DECL_OVERRIDE;

    private:
       Stats    _stats;
       QMutex   _mutex;
       bool     _lockState;
       bool     _connectionState;
};

#endif // STATSMONITOR_H
