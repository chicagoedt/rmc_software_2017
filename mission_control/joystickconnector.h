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

        inline const Axis&   axisLeft(void)  const { return _axisLeft; }
        inline const Axis&   axisRight(void) const { return _axisRight; }
                     eState  btnState(unsigned char buttonID) const;

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

            void     quit();

            bool     toggleInputLock();

     signals:
        void    deviceConnected(const QString& label);
        void    deviceDisconnected(void);
        void    deviceUpdate(const InputUpdate& state);
        void    deviceBtnUpdate( eBtnState state, int btnID );

        void    statusUpdate(const eStatus& status,
                             const QString& message);

     private:
        void            initialize(void);
        SDL_Joystick*   selectController(void);
        void            handleController(void);
        void            onControllerButtonEvent( const SDL_ControllerButtonEvent& event );
        void            onControllerAxisEvent( const SDL_ControllerAxisEvent& event );
        void            onJoystickAxisEvent( const SDL_JoyAxisEvent& event);
        void            onJoystickButtonEvent( const SDL_JoyButtonEvent& event);
        void            addControllerEvent( const SDL_ControllerDeviceEvent& event );
        void            removeControllerEvent( const SDL_ControllerDeviceEvent& event );
        void            addJoystickEvent( const SDL_JoyDeviceEvent& event);
        void            removeJoystickEvent( const SDL_JoyDeviceEvent& event);

     private:
        InputUpdate _currentState;
        bool        _lockState;

    protected:
       virtual void    run(void);// Q_DECL_OVERRIDE;
};


#endif // JOYSTICKCONNECTOR_H
