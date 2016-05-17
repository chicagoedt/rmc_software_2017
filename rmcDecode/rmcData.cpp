#include "rmcData.h"

bool    RMCData::operator!=(const RMCData& data) const
{
    if( _posX       != data.posX()   ||
        _posY       != data.posY()   ||
        _orient     != data.orient() ||
        _digState   != data.digState() )
        return true;
    else
        return false;
}

bool    RMCData::operator==(const RMCData& data) const
{
    if( _posX       == data.posX() &&
        _posY       == data.posY() &&
        _orient     == data.orient() &&
        _digState   == data.digState())
        return true;
    else
        return false;
}

