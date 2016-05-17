#include "rmcEnDecoder.h"

RMCEnDecoder::RMCEnDecoder(void)
{
}

// O - Orientation
// X - X Position
// Y - Y Position
// D - Dig state
// U - Unused bits
// <OOOOO XXX> <XXXXXX YY> <YYYYYYYY> <UUUUUU DD>
const RMCEnDecoder::TVec& RMCEnDecoder::encodeMessage(const RMCData& data)
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

    // <........> <........> <........> <......DD>
    _rawData[3]  = data.digState() & 0x03;

    if( _printDebug )
        cout << "OBits: " << std::bitset<8>(_rawData[0]).to_string() <<
                      " " << std::bitset<8>(_rawData[1]).to_string() <<
                      " " << std::bitset<8>(_rawData[2]).to_string() <<
                      " " << std::bitset<8>(_rawData[3]).to_string() << endl;

    return _rawData;
}

const RMCData&  RMCEnDecoder::decodeMessage(const RMCEnDecoder::TVec& rawData)
{
    return decodeMessage(rawData.buffer(), rawData.size());
}

// O - Orientation
// X - X Position
// Y - Y Position
// D - Dig state
// U - Unused bits
// <OOOOO XXX> <XXXXXX YY> <YYYYYYYY> <UUUUUU DD>
const RMCData&  RMCEnDecoder::decodeMessage(const unsigned char* pBuffer, unsigned int size)
{
    assert(size == MAX_MSG_SIZE);

    if( _printDebug )
        cout << "IBits: " << std::bitset<8>(pBuffer[0]).to_string() <<
                      " " << std::bitset<8>(pBuffer[1]).to_string() <<
                      " " << std::bitset<8>(pBuffer[2]).to_string() <<
                      " " << std::bitset<8>(pBuffer[3]).to_string() << endl;

    _rmcData._orient = (pBuffer[0] >> 3) & 0x1F;

    _rmcData._posX   = ((pBuffer[0] & 0x07) << 6) | ((pBuffer[1] & 0xFC) >> 2);
    _rmcData._posY   = ((pBuffer[1] & 0x03) << 8) | pBuffer[2];

    _rmcData._digState = static_cast<RMCData::eDigState>(pBuffer[3]);

    return _rmcData;
}
