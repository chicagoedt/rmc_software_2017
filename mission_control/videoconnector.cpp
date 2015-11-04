#include "videoconnector.h"

VideoConnector::VideoConnector(QObject* parent)
  : QThread(parent)
{

}

VideoConnector::~VideoConnector()
{

}

void    VideoConnector::run(void)
{
    while( QThread::currentThread()->isRunning() )
    {

    }
}
