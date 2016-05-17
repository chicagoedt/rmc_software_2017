#ifndef RMCENDECODER_H
#define RMCENDECODER_H

#include "rmcData.h"
#include <cassert>

class RMCEnDecoder
{
    public:

        class TVec
        {
            public:
                TVec(void) : _size(MAX_MSG_SIZE) {}

            inline unsigned int   size(void)   const { return _size;   }
            inline unsigned char* buffer(void) const { return const_cast<unsigned char*>(_buffer); }

            unsigned char operator[](int Idx) const
            {
                assert(Idx < MAX_MSG_SIZE);
                return _buffer[Idx];
            }

            unsigned char& operator[](int Idx)
            {
                assert(Idx < MAX_MSG_SIZE);
                return _buffer[Idx];
            }

            private:
                unsigned char _buffer[MAX_MSG_SIZE];
                unsigned int  _size;
        };

        RMCEnDecoder(void);

        inline void enableDebug(bool b) { _printDebug = b; }

        const RMCEnDecoder::TVec& encodeMesage(const RMCData& data);
        const RMCData&            decodeMesage(const RMCEnDecoder::TVec& rawData);
        const RMCData&            decodeMesage(const unsigned char* pBuffer, unsigned int size);

    private:
        RMCData _rmcData;
        TVec    _rawData;
        bool    _printDebug;
};


#endif // __RMCENDECODER_H__
