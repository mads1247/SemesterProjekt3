#include "Socket.h"
#include <iostream>
#include <ostream>


Socket::Socket(QObject *parent) : QObject(parent)
{

}

void Socket::connect(){

        socket->write("\n");

}

void Socket::connectMatlab(){

socket = new QTcpSocket(this);

socket->connectToHost("127.0.0.1", 30000);

if(socket->waitForConnected(3000))
{
    qDebug() << "Connected to Matlab!";

}
else{
    qDebug() << "Not connected!";
}

}

void Socket::writeMatlab(double input1)
{

    uint8_t* in1 = (uint8_t*)(&input1);
    char* res;


    for (int i = 0; i < sizeof(input1); ++i) {
        res[i] = in1[i];
    }


    socket->write(res);
    //socket->write("\n");
    socket->waitForBytesWritten(2000);
}

std::string Socket::readMatlab()
{
    socket->waitForReadyRead(3000);
    std::string s = (std::string) socket->readAll();

    return s;
}

void Socket::disconnectMatlab()
{
        socket->waitForReadyRead(2000);
        socket->close();
}



