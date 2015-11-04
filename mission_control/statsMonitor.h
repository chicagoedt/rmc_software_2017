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
           Resert();
        }

        inline uint    TxTotalBytes() const { return _txTotalBytes; }
        inline uint    TxBytesPerSec() const { return _txBytesPerSec; }
        inline uint    TxPacketPerSec() const { return _txPacketPerSec; }

        inline uint    RxTotalBytes() const { return _rxTotalBytes; }
        inline uint    RxBytesPerSec() const { return _rxBytesPerSec; }
        inline uint    RxPacketPerSec() const { return _rxPacketPerSec; }

    private:
        inline void    Resert()
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

        void    ResetStats();
        void    ToggleInputLock(bool state);
        void    ToggleConnectionState(bool state);

    signals:
        void    StatusUpdate(const eStatus& status, const QString& message);
        void    StatsUpdate(const Stats& stats);

    public slots:
        void    UpdateTxStats(const QByteArray& buffer);

    protected:
       virtual void    run(void);// Q_DECL_OVERRIDE;

    private:
       Stats    _stats;
       QMutex   _mutex;
       bool     _lockState;
       bool     _connectionState;
};

#endif // STATSMONITOR_H
