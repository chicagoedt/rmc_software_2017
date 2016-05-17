#include "rmcEnDecoder.h"

RMCEnDecoder::RMCEnDecoder(void)
{
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
    //unsigned char    buffer[MAX_MSG_SIZE] = {0};

    //for(unsigned int Idx = 0; Idx < rawData._size && Idx < MAX_MSG_SIZE; ++Idx)
    //    buffer[Idx] = rawData[Idx];

    if( _printDebug )
        cout << "IBits: " << std::bitset<8>(rawData[0]).to_string() <<
                      " " << std::bitset<8>(rawData[1]).to_string() <<
                      " " << std::bitset<8>(rawData[2]).to_string() << endl;

    return decodeMesage(rawData.buffer(), rawData.size());
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
