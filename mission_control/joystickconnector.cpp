#include "joystickconnector.h"
#include <QDebug>

InputUpdate::eState    InputUpdate::btnState(unsigned char buttonID) const
{
    switch(buttonID)
    {
        case 1:
            return static_cast<eState>( _btnState & 0x01);
        case 2:
            return static_cast<eState>( _btnState & 0x02);
        case 3:
            return static_cast<eState>( _btnState & 0x04);
        case 4:
            return static_cast<eState>( _btnState & 0x08);
        case 5:
            return static_cast<eState>( _btnState & 0x10);
        case 6:
            return static_cast<eState>( _btnState & 0x20);
        case 7:
            return static_cast<eState>( _btnState & 0x40);
        case 8:
            return static_cast<eState>( _btnState & 0x80);

        default:
            return eOFF;
    }
}

JoystickConnector::JoystickConnector(QObject* parent)
    : QThread(parent), DEAD_ZONE(8000), _lockState(true)
{

}

JoystickConnector::~JoystickConnector()
{

}

void    JoystickConnector::quit()
{
    terminate();

    // Push event to wake up so thread can exit
    SDL_Event sdlevent;
    sdlevent.type = SDL_KEYDOWN;
    sdlevent.key.keysym.sym = SDLK_1;

    SDL_PushEvent(&sdlevent);

    wait();
}

bool    JoystickConnector::toggleInputLock()
{
    _lockState = ! _lockState;

    return _lockState;
}

void    JoystickConnector::run(void)
{
    emit statusUpdate( eInfo, QString("Joystick thread initialized."));

    while( QThread::currentThread()->isRunning() )
    {
        initialize();

        SDL_Joystick* pGameController = selectController();

        if( pGameController )
        {
            handleController();

            SDL_JoystickClose( pGameController );
            pGameController = 0L;

            emit deviceDisconnected();
            QThread::msleep(100);
        }
        else
        {
            QThread::msleep(200);
        }

        SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
    }

    emit statusUpdate( eInfo, QString("Joystick thread terminated."));
}

void    JoystickConnector::initialize(void)
{
    bool logOnce(true);

    while( QThread::currentThread()->isRunning() )
    {
        if( SDL_InitSubSystem( SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER ) < 0 )
        {
            if( logOnce )
            {
                logOnce = false;
                emit statusUpdate( eError, QString("SDL init failed. SDL Error: %1")
                                                         .arg( SDL_GetError()));
            }

            QThread::msleep(200);
            continue;
        }

        if( QThread::currentThread()->isRunning() )
        {
            SDL_JoystickEventState(SDL_ENABLE);
            emit statusUpdate( eInfo, QString("Input system initialized."));
            break;
        }
    }
}

SDL_Joystick* JoystickConnector::selectController(void)
{
    //Check for joysticks
    if( SDL_NumJoysticks() > 0 )
    {
        for( int Idx(0); Idx < SDL_NumJoysticks(); ++Idx)
        {
            SDL_Joystick* pGameController = SDL_JoystickOpen( Idx );
            //Load joystick
            if( pGameController != 0L )
            {
                emit deviceConnected( SDL_JoystickName(pGameController));
                return pGameController;
            }
        }

        emit statusUpdate( eError, QString("Unable to open game controller. Found: %s").arg(SDL_NumJoysticks()) );
    }
    else
        emit statusUpdate( eError, QString("Warning: No joysticks connected. Scanning") );

    return 0L;
}

void    JoystickConnector::handleController(void)
{
    //Event handler
    SDL_Event event;

    while( QThread::currentThread()->isRunning() )
    {
        //if( SDL_PollEvent( &event ) )
        if( SDL_WaitEvent(&event ) )
        {
            if( !( event.type == SDL_JOYBUTTONDOWN && event.jbutton.button == 0) )
            {
                if( _lockState )
                    continue;
            }

            switch( event.type )
            {
                case SDL_CONTROLLERBUTTONDOWN:
                case SDL_CONTROLLERBUTTONUP:
                    onControllerButtonEvent( event.cbutton );
                    break;

                case SDL_CONTROLLERAXISMOTION:
                    onControllerAxisEvent( event.caxis );
                    break;

                case SDL_JOYAXISMOTION:
                    onJoystickAxisEvent(event.jaxis);
                    break;

                case SDL_JOYBUTTONUP:
                case SDL_JOYBUTTONDOWN:
                    onJoystickButtonEvent(event.jbutton);
                    break;

                case SDL_CONTROLLERDEVICEADDED:
                    addControllerEvent( event.cdevice );
                    break;

                case SDL_CONTROLLERDEVICEREMOVED:
                    removeControllerEvent( event.cdevice );
                    break;

                case SDL_JOYDEVICEADDED:
                    addJoystickEvent( event.jdevice );
                    break;

                case SDL_JOYDEVICEREMOVED:
                    removeJoystickEvent( event.jdevice );
                    break;

                default:
                    break;
            }
        }
    }
}

void    JoystickConnector::onJoystickAxisEvent( const SDL_JoyAxisEvent& event)
{

    if( event.axis == SDL_CONTROLLER_AXIS_LEFTX )
    {
        if( qAbs<Sint32>(event.value) < DEAD_ZONE)
            _currentState._axisLeft._x = 0;
        else
            _currentState._axisLeft._x = event.value;
        //qDebug() << "AXIS_LEFTX " << event.value << _currentState._axisLeft._x;
    }

    if( event.axis == SDL_CONTROLLER_AXIS_LEFTY )
    {
        if( qAbs<Sint32>(event.value) < DEAD_ZONE)
            _currentState._axisLeft._y = 0;
        else if(qAbs<Sint32>(event.value) > 32767 )
        {
            _currentState._axisLeft._y = 32767;
           //qDebug() << "Editing weird val " << event.value << "to" << _currentState._axisLeft._y << endl;
        }
        else
            _currentState._axisLeft._y = -event.value;
        //qDebug() << "AXIS_LEFTY " << event.value << _currentState._axisLeft._y;
    }

    else if( event.axis == SDL_CONTROLLER_AXIS_RIGHTX )
    {
        if( qAbs<Sint32>(event.value) < DEAD_ZONE)
            _currentState._axisRight._x = 0;
        else
            _currentState._axisRight._x = event.value;

        //qDebug() << "AXIS_RIGHTX " << event.value << _currentState._axisRight._x;
    }


    else if(event.axis == SDL_CONTROLLER_AXIS_RIGHTY)
    {
        if( qAbs<Sint32>(event.value) < DEAD_ZONE)
        {
            //qDebug() << "Doing dumb shit " << event.value << _currentState._axisRight._y;
            _currentState._axisRight._y = 0;
        }
        else if(qAbs<Sint32>(event.value) > 32767 )
        {
            _currentState._axisRight._y = 32767;
           //qDebug() << "Editing weird val " << event.value << "to" << _currentState._axisLeft._y << endl;
        }
        else
            _currentState._axisRight._y = -event.value;

        //qDebug() << "AXIS_RIGHTY " << event.value << _currentState._axisRight._y;
    }



/*
    else{
        if( qAbs<Sint32>(event.value) < DEAD_ZONE_LEFT)
            _currentState._axisRight._y = 0;
        else if(qAbs<Sint32>(event.value) > 32767 )
        {
            _currentState._axisRight._y = 32767;
           //qDebug() << "Editing weird val " << event.value << "to" << _currentState._axisLeft._y << endl;
        }
        else
            _currentState._axisRight._y = -event.value;
        qDebug() << "AXIS_RIGHTX " << event.value << _currentState._axisRight._x;
    }
    */
    emit deviceUpdate( _currentState );
}

void    JoystickConnector::onJoystickButtonEvent( const SDL_JoyButtonEvent& event)
{
    if( event.type == SDL_JOYBUTTONDOWN)
        emit deviceBtnUpdate( eDown, event.button );
    else if( event.type == SDL_JOYBUTTONUP)
        emit deviceBtnUpdate( eUp, event.button );
}

void    JoystickConnector::onControllerButtonEvent( const SDL_ControllerButtonEvent& )
{
    qDebug() << "OnControllerButtonEvent";
}

void    JoystickConnector::onControllerAxisEvent( const SDL_ControllerAxisEvent& )
{
    qDebug() << "OnControllerAxisEvent";
}

void    JoystickConnector::addControllerEvent( const SDL_ControllerDeviceEvent& )
{
    qDebug() << "AddControllerEvent";
}

void    JoystickConnector::removeControllerEvent( const SDL_ControllerDeviceEvent& )
{
    qDebug() << "RemoveControllerEvent";
}

void    JoystickConnector::addJoystickEvent( const SDL_JoyDeviceEvent& )
{
    qDebug() << "AddJoystickEvent";
}

void    JoystickConnector::removeJoystickEvent( const SDL_JoyDeviceEvent& )
{
    qDebug() << "RemoveJoystickEvent";
}

