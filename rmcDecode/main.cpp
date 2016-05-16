#include <iostream>
#include <vector>
#include <cassert>

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

bool    RMCData::operator!=(const RMCData& data) const
{
    if( _posX != data.posX() ||
        _posY != data.posY() ||
        _orient != data.orient() )
        return true;
    else
        return false;
}

bool    RMCData::operator==(const RMCData& data) const
{
    if( _posX == data.posX() &&
        _posY == data.posY() &&
        _orient == data.orient() )
        return true;
    else
        return false;
}

std::ostream& operator <<(std::ostream &os, const RMCData& obj)
{
    return os << "X: " << obj.posX() << " Y: " << obj.posY() << " O: " << obj.orient();
}

class RMCEnDecoder
{
    public:
        typedef std::vector<unsigned char> TVec;

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

RMCEnDecoder::RMCEnDecoder(void)
{
    _rawData.resize(MAX_MSG_SIZE);
}

// <OOOOO XXX> <XXXXXX YY> <YYYYYYYY>
const RMCEnDecoder::TVec& RMCEnDecoder::encodeMesage(const RMCData& data)
{
    // <OOOOO...> <........> <........>
    _rawData[0] = (unsigned char)((data.orient() & 0x1F) << 3);

    // <.....XXX> <........> <........>
    _rawData[0] |= (data.posX() & 0x01C0) >> 6;
    // <........> <XXXXXX..> <........>
    _rawData[1]  = (data.posX() & 0x3F) << 2;

    // <........> <......YY> <........>
    _rawData[1] |= (data.posY() & 0x300) >> 8;

    // <........> <........> <YYYYYYYY>
    _rawData[2]  = (data.posY() & 0xFF);

    std::bitset<8> byte0(_rawData[0]);
    std::bitset<8> byte1(_rawData[1]);
    std::bitset<8> byte2(_rawData[2]);

    if( _printDebug )
        cout << "OBits: " << std::bitset<8>(_rawData[0]).to_string() <<
                      " " << std::bitset<8>(_rawData[1]).to_string() <<
                      " " << std::bitset<8>(_rawData[2]).to_string() << endl;

    return _rawData;
}

const RMCData&  RMCEnDecoder::decodeMesage(const RMCEnDecoder::TVec& rawData)
{
    unsigned char    buffer[MAX_MSG_SIZE] = {0};

    for(unsigned int Idx = 0; Idx < rawData.size() && Idx < MAX_MSG_SIZE; ++Idx)
        buffer[Idx] = rawData[Idx];

    if( _printDebug )
        cout << "IBits: " << std::bitset<8>(buffer[0]).to_string() <<
                      " " << std::bitset<8>(buffer[1]).to_string() <<
                      " " << std::bitset<8>(buffer[2]).to_string() << endl;

    return decodeMesage(buffer, MAX_MSG_SIZE);
}

// <OOOOO XXX> <XXXXXX YY> <YYYYYYYY>
const RMCData&  RMCEnDecoder::decodeMesage(const unsigned char* pBuffer, unsigned int size)
{
    assert(size == MAX_MSG_SIZE);

    _rmcData._orient = (pBuffer[0] >> 3) & 0x1F;

    _rmcData._posX   = ((pBuffer[0] & 0x07) << 6) | ((pBuffer[1] & 0xFC) >> 2);
    _rmcData._posY   = ((pBuffer[1] & 0x03) << 8) | pBuffer[2];

    return _rmcData;
}

void    test(bool debug, unsigned int testID, int x, int y, int orient)
{
    RMCEnDecoder    rmcEnDecoder;

    rmcEnDecoder.enableDebug(debug);

    RMCData         rmcData(x, y, orient);

    const RMCEnDecoder::TVec& encodedData = rmcEnDecoder.encodeMesage(rmcData);
    const RMCData&            decodedData = rmcEnDecoder.decodeMesage(encodedData);

    if( debug )
    {
        cout << "Test[" << testID << "]" << endl;
        cout << "\t I X: " << x << " Y: " << y << " O: " << orient << endl;
        cout << "\t O " <<  decodedData << endl;
    }

    if( rmcData != decodedData)
        cout << "!!! Test Failed !!!" << endl;

    if( debug )
        cout << endl;
}

// Size of arena
// 378 cm, 738 cm
int main(int argc, char *argv[])
{
    bool printDebug(false);

    if( argc > 1)
        printDebug = true;

    unsigned testId = 1;

    for(int Idx = 0; Idx < 378; ++Idx )
        for(int Idy = 0; Idy < 738; ++Idy )
            for(int orient = 0; orient < 31; ++orient )
            {
                test(printDebug, testId, Idx, Idy, orient);
                ++testId;
            }

    return 0;
}


