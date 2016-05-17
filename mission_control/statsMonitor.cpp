#include "statsMonitor.h"

StatsMonitor::StatsMonitor(QObject* parent)
: QThread(parent), _lockState(true), _connectionState(false)
{

}

void    StatsMonitor::resetStats()
{
    _mutex.lock();
        _stats.reset();
    _mutex.unlock();
}

void    StatsMonitor::toggleInputLock(bool state)
{
    _lockState = state;
}

void    StatsMonitor::toggleConnectionState(bool state)
{
    _connectionState = state;
}

void    StatsMonitor::updateTxStats(const QByteArray& buffer)
{
    if( _connectionState )
    {
        _mutex.lock();

            _stats._txTotalBytes  += (uint)buffer.size();
            _stats._txBytesPerSec += (uint)buffer.size();
            _stats._txPacketPerSec++;

        _mutex.unlock();
    }
}

void    StatsMonitor::run(void)
{
    emit statusUpdate( eInfo, QString("Stats Monitor thread initialized"));

    while( QThread::currentThread()->isRunning() )
    {
        if( _lockState == false )
        {
            _mutex.lock();
                emit statsUpdate(_stats);
                _stats._txBytesPerSec  = 0;
                _stats._txPacketPerSec = 0;
            _mutex.unlock();
        }

        QThread::msleep(1000);
    }

    emit statusUpdate( eInfo, QString("Stats Monitor thread terminated"));
}
