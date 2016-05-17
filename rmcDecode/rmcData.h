#ifndef RMCDATA_H
#define RMCDATA_H

#include <iostream>

using namespace std;

#define MAX_MSG_SIZE    4

class RMCData
{
    friend class RMCEnDecoder;

    public:

        enum eDigState
        {
            eDigState_Dig = 0,
            eDigState_Home,
            eDigState_Up
        };

        RMCData(void)
            : _posX(0), _posY(0), _orient(0), _digState(eDigState_Home) {}
        RMCData(int X, int Y, int orient, eDigState state)
            : _posX(X), _posY(Y), _orient(orient), _digState(state) {}

        inline int          posX(void)    const { return _posX;   }
        inline int          posY(void)    const { return _posY;   }
        inline int          orient(void)  const { return _orient; }
        inline eDigState    digState(void)     const { return _digState; }

        bool    operator==(const RMCData& data) const;
        bool    operator!=(const RMCData& data) const;

    private:
        int         _posX;
        int         _posY;
        int         _orient;
        eDigState   _digState;

};

#endif // RMCDATA_H
