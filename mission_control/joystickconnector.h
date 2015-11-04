#ifndef JOYSTICKCONNECTOR_H
#define JOYSTICKCONNECTOR_H

#include <QThread>
#include <SDL2/SDL.h>
#include "commonhdr.h"

class InputUpdate
{
        friend class JoystickConnector;

    public:
        struct Axis
        {
            friend class JoystickConnector;

            public:
                Axis(void) : _x(0), _y(0) {}

                inline Sint16 X(void) const { return _x; }
                inline Sint16 Y(void) const { return _y; }

            private:
                Sint16 _x;
                Sint16 _y;
        };

        InputUpdate(void) : _btnState(0) {}

        enum eState
        {
           eOFF,
           eON
        };

        inline const Axis&   AxisLeft(void)  const { return _axisLeft; }
        inline const Axis&   AxisRight(void) const { return _axisRight; }
                     eState  BtnState(unsigned char buttonID) const;

    private:
        Axis          _axisLeft;
        Axis          _axisRight;
        unsigned char _btnState;
};

class JoystickConnector : public QThread
{
    Q_OBJECT

        const int DEAD_ZONE;

    public:
            explicit JoystickConnector(QObject* parent = 0L);
            virtual ~JoystickConnector();

            void     Quit();

            bool     ToggleInputLock();

     signals:
        void    DeviceConnected(const QString& label);
        void    DeviceDisconnected(void);
        void    DeviceUpdate(const InputUpdate& state);
        void    DeviceBtnUpdate( eBtnState state, int btnID );

        void    StatusUpdate(const eStatus& status,
                             const QString& message);

     private:
        void            Initialize(void);
        SDL_Joystick*   SelectController(void);
        void            HandleController(void);
        void            OnControllerButtonEvent( const SDL_ControllerButtonEvent& event );
        void            OnControllerAxisEvent( const SDL_ControllerAxisEvent& event );
        void            OnJoystickAxisEvent( const SDL_JoyAxisEvent& event);
        void            OnJoystickButtonEvent( const SDL_JoyButtonEvent& event);
        void            AddControllerEvent( const SDL_ControllerDeviceEvent& event );
        void            RemoveControllerEvent( const SDL_ControllerDeviceEvent& event );
        void            AddJoystickEvent( const SDL_JoyDeviceEvent& event);
        void            RemoveJoystickEvent( const SDL_JoyDeviceEvent& event);

     private:
        InputUpdate _currentState;
        bool        _lockState;

    protected:
       virtual void    run(void);// Q_DECL_OVERRIDE;
};


#endif // JOYSTICKCONNECTOR_H
