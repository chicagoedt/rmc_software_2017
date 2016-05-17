#include "rmcEnDecoder.h"

std::ostream& operator <<(std::ostream &os, const RMCData& obj)
{
    return os << "X: " << obj.posX() << " Y: " << obj.posY() << " O: " << obj.orient();
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


