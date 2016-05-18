#ifndef INPUTTHROTTLER_H
#define INPUTTHROTTLER_H

#include <joystickconnector.h>
#include <QString>
#include <QMutex>

#define JOY_RANGE               32767           //Max value of Movement axes
#define BITS_PER_MSG_WHEELE     4               // 3 + signe bit
#define JOY_PER_MSG_SCALAR      (short)(JOY_RANGE / 7)

class InputThrottler : public QThread
{
    Q_OBJECT
    public:

        enum eOperationMode // 3 States the gui can be in
        {
            eSafety = 0,
            eAuto,
            eManual
        };

        explicit InputThrottler(QObject* parent = 0L);
        virtual ~InputThrottler();

        void    Initialize(void);

        void    SetMode(const eOperationMode mode);
        void    SetMaxUpdateRate(unsigned int upsMax);

    public slots:
        void    DeviceUpdate(const InputUpdate& state);
        void    UpdateRateChanged(unsigned int ups);
        void    DeviceBtnUpdate( eBtnState state,
                                 int btnID );

    signals:
        void    StatusUpdate(const eStatus& status, const QString& message);
        void    BitsUpdate(const QString& message);
        void    PublishMessage(const QByteArray& buffer);
        void    ActuatorState( int level );
        void    DiggingState(bool enabled);
        void    EStopUpdate(bool enable);

    private:
        void    PackBits();
        void    PrintBits();
        void    CalculateSleepPeriod();

    private:
        InputUpdate     _state;
        eOperationMode  _mode;
        QByteArray      _byteArray;
        QMutex          _lock;
        int             _actuatorLevel;
        bool            _updated;
        bool            _digging;
        bool            _eStop;
        unsigned int    _updatesMaxPerSecRate;
        unsigned int    _updatesPerSecRate;
        unsigned int    _sleepInterval;

    protected:
       virtual void    run(void);// Q_DECL_OVERRIDE;
};

#endif // INPUTTHROTTLER_H
