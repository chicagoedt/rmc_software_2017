#include "statsMonitor.h"

StatsMonitor::StatsMonitor(QObject* parent)
: QThread(parent), _lockState(true), _connectionState(false)
{

}

void    StatsMonitor::ResetStats()
{
    _mutex.lock();
        _stats.Resert();
    _mutex.unlock();
}

void    StatsMonitor::ToggleInputLock(bool state)
{
    _lockState = state;
}

void    StatsMonitor::ToggleConnectionState(bool state)
{
    _connectionState = state;
}

void    StatsMonitor::UpdateTxStats(const QByteArray& buffer)
{
    if( _connectionState )
    {
        _mutex.lock();

            _stats._txTotalBytes  += buffer.size();
            _stats._txBytesPerSec += buffer.size();
            _stats._txPacketPerSec++;

        _mutex.unlock();
    }
}

void    StatsMonitor::run(void)
{
    emit StatusUpdate( eOK, QString("Stats Monitor thread initialized"));

    while( QThread::currentThread()->isRunning() )
    {
        if( _lockState == false )
        {
            _mutex.lock();
                emit StatsUpdate(_stats);
                _stats._txBytesPerSec  = 0;
                _stats._txPacketPerSec = 0;
            _mutex.unlock();
        }

        QThread::msleep(1000);
    }

    emit StatusUpdate( eOK, QString("Stats Monitor thread terminated"));
}
