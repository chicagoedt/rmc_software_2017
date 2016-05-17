#ifndef RMCDATA_H
#define RMCDATA_H

#include <iostream>

using namespace std;

#define MAX_MSG_SIZE    3

class RMCData
{
    friend class RMCEnDecoder;

    public:
        RMCData(void)
            : _posX(0), _posY(0), _orient(0) {}
        RMCData(int X, int Y, int oritnt)
            : _posX(X), _posY(Y), _orient(oritnt) {}

        inline int     posX(void)       const { return _posX;   }
        inline int     posY(void)       const { return _posY;   }
        inline int     orient(void)     const { return _orient; }

        bool    operator==(const RMCData& data) const;
        bool    operator!=(const RMCData& data) const;

    private:
        int    _posX;
        int    _posY;
        int    _orient;
};

#endif // RMCDATA_H
