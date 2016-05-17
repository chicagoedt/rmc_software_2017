#include "rmcEnDecoder.h"

std::ostream& operator <<(std::ostream &os, const RMCData& obj)
{
    return os << "X: " << obj.posX() << " Y: " << obj.posY()
              << " O: " << obj.orient() << " D: " << static_cast<int>(obj.digState());
}

void    test(bool debug, unsigned int testID, int x, int y, int orient, RMCData::eDigState state)
{
    RMCEnDecoder    rmcEnDecoder;

    rmcEnDecoder.enableDebug(debug);

    RMCData         rmcData(x, y, orient, state);

    const RMCEnDecoder::TVec& encodedData = rmcEnDecoder.encodeMessage(rmcData);
    const RMCData&            decodedData = rmcEnDecoder.decodeMessage(encodedData);

    if( debug )
    {
        cout << "Test[" << testID << "]" << endl;
        cout << "\t I X: " << x << " Y: " << y << " O: " << orient << " D: " << static_cast<int>(state) << endl;
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
                for(int dig = 0; dig <= RMCData::eDigState_Up; ++dig)
                {
                    test(printDebug, testId, Idx, Idy, orient, static_cast<RMCData::eDigState>(dig));
                    ++testId;
                }

    return 0;
}


