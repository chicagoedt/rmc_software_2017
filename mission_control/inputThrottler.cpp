#include "inputThrottler.h"
#include <QBitArray>
#include <QDebug>

//This file deals with the actuator/digging controls.

InputThrottler::InputThrottler(QObject* parent)
    : QThread(parent), _mode(eSafety), _actuatorLevel(0),
      _updated(false), _digging(false), _updatesMaxPerSecRate(50),
      _updatesPerSecRate(10), _sleepInterval(100)
{
    _byteArray.reserve(2);
}

InputThrottler::~InputThrottler()
{

}

void    InputThrottler::run(void)
{
    emit StatusUpdate( eInfo, QString("Input Throttler thread initialized"));

    while( QThread::currentThread()->isRunning() )
    {
        _lock.lock();

        if(_updated )
        {
            _updated = false;
            _lock.unlock();

            PackBits();
            PrintBits();

            emit PublishMessage(_byteArray);
        }
        else
            _lock.unlock();

        QThread::msleep(_sleepInterval);
    }

    emit StatusUpdate( eInfo, QString("Input Throttler thread terminated"));
}

void    InputThrottler::SetMode(const eOperationMode mode)
{
    _lock.lock();

    _updated = true;
    _mode    = mode;

    _lock.unlock();

    emit PublishMessage(_byteArray);
}

void    InputThrottler::PackBits()
{
    _byteArray.clear();

    _byteArray[0] = (char)_mode |
                    ((_actuatorLevel << 3) & 0x00FF) |
                    (char)_digging << 2;
    _byteArray[1] = ((_state.axisRight().Y() / JOY_PER_MSG_SCALAR) & 0x0F) |
                    ((_state.axisLeft().Y() / JOY_PER_MSG_SCALAR) & 0x0F) << 4;
}
void    InputThrottler::DeviceUpdate(const InputUpdate& state)
{
    _lock.lock();

    _state   = state;
    _updated = true;

    _lock.unlock();
}

void    InputThrottler::SetMaxUpdateRate(unsigned int upsMax)
{
    if( upsMax == 0)
    {
        _updatesMaxPerSecRate = 2;
        _updatesPerSecRate    = 1;
    }
    else if(upsMax > 100)   // No higher then 100 Updates per seconds
        _updatesMaxPerSecRate = upsMax;
    else
    {
        _updatesMaxPerSecRate = upsMax;

        if( _updatesMaxPerSecRate < _updatesPerSecRate)
            _updatesPerSecRate = _updatesMaxPerSecRate;
    }

    CalculateSleepPeriod();
}

void    InputThrottler::CalculateSleepPeriod()
{
    _sleepInterval = 1000 / _updatesPerSecRate;
}

void    InputThrottler::UpdateRateChanged(unsigned int ups)
{
    if( ups > _updatesMaxPerSecRate)
        _updatesPerSecRate = _updatesMaxPerSecRate;
    else if (ups < 1 )
        _updatesPerSecRate = 1;
    else
        _updatesPerSecRate = ups;

    CalculateSleepPeriod();
}

void    InputThrottler::DeviceBtnUpdate( eBtnState state, int btnID )
{
    //qDebug() << "Button " << btnID;

    if( state == eDown )
    {
        /*
        if( btnID == 6 )    //Decrease Actuator Level
        {
            // Down
            if( _actuatorLevel == 0)
                return;

            _lock.lock();

            --_actuatorLevel;
            _updated = true;

            emit ActuatorState(_actuatorLevel);

            _lock.unlock();
        }
        else if( btnID == 7 )    //Increase Actuator Level
        {
            // Up
            if( _actuatorLevel == 3)
                return;

            _lock.lock();

            ++_actuatorLevel;
            _updated = true;

            emit ActuatorState(_actuatorLevel);

            _lock.unlock();
        }
        */
        if( btnID == 3 )    // Up
        {
            if( _actuatorLevel == 2)
                return;

            _lock.lock();

            _actuatorLevel = 2;
            _updated = true;

            emit ActuatorState(_actuatorLevel);

            _lock.unlock();
        }
        else if( btnID == 2 )    // Home
        {
            if( _actuatorLevel == 0)
                return;

            _lock.lock();

            _actuatorLevel = 0;

            _updated = true;

            emit ActuatorState(_actuatorLevel);

            _lock.unlock();
        }
        else if( btnID == 1 )    // Down
        {
            if( _actuatorLevel == 1)
                return;

            _lock.lock();

            _actuatorLevel = 1;

            _updated = true;

            emit ActuatorState(_actuatorLevel);

            _lock.unlock();
        }
        else if( btnID == 6)
        {
            _lock.lock();

            _digging = !_digging;
            _updated = true;
            emit DiggingState(_digging);

            _lock.unlock();
        }
    }
}

void    InputThrottler::PrintBits()
{
    QString msg;

    // Convert from QByteArray to QBitArray
    for(int i = 0; i <_byteArray.count() ; ++i)
    {
        for(int b(0); b < 8; ++b)
            msg.append( (_byteArray.at(i) & (1<<(7-b))) ? '1' : '0');

        if( i < _byteArray.count() )
            msg.append(" ");
    }

    emit BitsUpdate(msg);
}
