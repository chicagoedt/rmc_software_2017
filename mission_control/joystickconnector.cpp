#include "joystickconnector.h"
#include <QDebug>

InputUpdate::eState    InputUpdate::BtnState(unsigned char buttonID) const
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

void    JoystickConnector::Quit()
{
    terminate();

    // Push event to wake up so thread can exit
    SDL_Event sdlevent;
    sdlevent.type = SDL_KEYDOWN;
    sdlevent.key.keysym.sym = SDLK_1;

    SDL_PushEvent(&sdlevent);

    wait();
}

bool    JoystickConnector::ToggleInputLock()
{
    _lockState = ! _lockState;

    return _lockState;
}

void    JoystickConnector::run(void)
{
    emit StatusUpdate( eOK, QString("Joystick thread initialized."));

    while( QThread::currentThread()->isRunning() )
    {
        Initialize();

        SDL_Joystick* pGameController = SelectController();

        if( pGameController )
        {
            HandleController();

            SDL_JoystickClose( pGameController );
            pGameController = 0L;

            emit DeviceDisconnected();
            QThread::msleep(100);
        }
        else
        {
            QThread::msleep(200);
        }

        SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
    }

    emit StatusUpdate( eOK, QString("Joystick thread terminated."));
}

void    JoystickConnector::Initialize(void)
{
    bool logOnce(true);

    while( QThread::currentThread()->isRunning() )
    {
        if( SDL_InitSubSystem( SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER ) < 0 )
        {
            if( logOnce )
            {
                logOnce = false;
                emit StatusUpdate( eERROR, QString("SDL init failed. SDL Error: %1")
                                                         .arg( SDL_GetError()));
            }

            QThread::msleep(200);
            continue;
        }

        if( QThread::currentThread()->isRunning() )
        {
            SDL_JoystickEventState(SDL_ENABLE);
            emit StatusUpdate( eOK, QString("Input system initialized."));
            break;
        }
    }
}

SDL_Joystick* JoystickConnector::SelectController(void)
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
                emit DeviceConnected( SDL_JoystickName(pGameController));
                return pGameController;
            }
        }

        emit StatusUpdate( eERROR, QString("Unable to open game controller. Found: %s").arg(SDL_NumJoysticks()) );
    }
    else
        emit StatusUpdate( eERROR, QString("Warning: No joysticks connected. Scanning") );

    return 0L;
}

void    JoystickConnector::HandleController(void)
{
    //Event handler
    SDL_Event event;

    while( QThread::currentThread()->isRunning() )
    {
        //if( SDL_PollEvent( &event ) )
        if( SDL_WaitEvent(&event ) )
        {
            if( !( event.type == SDL_JOYBUTTONDOWN && event.jbutton.button == 2) )
            {
                if( _lockState )
                    continue;
            }

            switch( event.type )
            {
                case SDL_CONTROLLERBUTTONDOWN:
                case SDL_CONTROLLERBUTTONUP:
                    OnControllerButtonEvent( event.cbutton );
                    break;

                case SDL_CONTROLLERAXISMOTION:
                    OnControllerAxisEvent( event.caxis );
                    break;

                case SDL_JOYAXISMOTION:
                    OnJoystickAxisEvent(event.jaxis);
                    break;

                case SDL_JOYBUTTONUP:
                case SDL_JOYBUTTONDOWN:
                    OnJoystickButtonEvent(event.jbutton);
                    break;

                case SDL_CONTROLLERDEVICEADDED:
                    AddControllerEvent( event.cdevice );
                    break;

                case SDL_CONTROLLERDEVICEREMOVED:
                    RemoveControllerEvent( event.cdevice );
                    break;

                case SDL_JOYDEVICEADDED:
                    AddJoystickEvent( event.jdevice );
                    break;

                case SDL_JOYDEVICEREMOVED:
                    RemoveJoystickEvent( event.jdevice );
                    break;

                default:
                    break;
            }
        }
    }
}

void    JoystickConnector::OnJoystickAxisEvent( const SDL_JoyAxisEvent& event)
{
    if( event.axis == SDL_CONTROLLER_AXIS_LEFTX )
    {
        if( qAbs<Sint32>(event.value) < DEAD_ZONE)
            _currentState._axisLeft._x = 0;
        else
            _currentState._axisLeft._x = event.value;
        //qDebug() << "AXIS_LEFTX " << event.value << _currentState._axisLeft._x;
    }
    else  if( event.axis == SDL_CONTROLLER_AXIS_LEFTY )
    {
        if( qAbs<Sint32>(event.value) < DEAD_ZONE)
            _currentState._axisLeft._y = 0;
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
            _currentState._axisRight._y = 0;
        else
            _currentState._axisRight._y = event.value;

        //qDebug() << "AXIS_RIGHTY " << event.value << _currentState._axisRight._y;
    }

    //currentState._axisLeft._x << _currentState._axisLeft._y << "||" <<
                //_currentState._axisRight._x << _currentState._axisRight._y;
    emit DeviceUpdate( _currentState );
}

void    JoystickConnector::OnJoystickButtonEvent( const SDL_JoyButtonEvent& event)
{
    if( event.type == SDL_JOYBUTTONDOWN)
        emit DeviceBtnUpdate( eDown, event.button );
    else if( event.type == SDL_JOYBUTTONUP)
        emit DeviceBtnUpdate( eUp, event.button );
}

void    JoystickConnector::OnControllerButtonEvent( const SDL_ControllerButtonEvent& )
{
    //qDebug() << "OnControllerButtonEvent";
}

void    JoystickConnector::OnControllerAxisEvent( const SDL_ControllerAxisEvent& )
{
    //qDebug() << "OnControllerAxisEvent";
}

void    JoystickConnector::AddControllerEvent( const SDL_ControllerDeviceEvent& )
{
    //qDebug() << "AddControllerEvent";
}

void    JoystickConnector::RemoveControllerEvent( const SDL_ControllerDeviceEvent& )
{
    //qDebug() << "RemoveControllerEvent";
}

void    JoystickConnector::AddJoystickEvent( const SDL_JoyDeviceEvent& )
{
    //qDebug() << "AddJoystickEvent";
}

void    JoystickConnector::RemoveJoystickEvent( const SDL_JoyDeviceEvent& )
{
    //qDebug() << "RemoveJoystickEvent";
}

