#include <QCoreApplication>
#include "cam_server.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    Server mServer;

    return a.exec();
}
