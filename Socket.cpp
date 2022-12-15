#include "Socket.h"
#include <iostream>
#include <ostream>
#include <cstring>


Socket::Socket(QObject *parent) : QObject(parent)
{

}

void Socket::connect(){

    socket = new QTcpSocket(this);

    socket->connectToHost("192.168.100.10", portNR);

    if(socket->waitForConnected(3000))
    {
        qDebug() << "Connected!";

        //Verbose
        socket->write("VERBOSE=1    \n");
        socket->waitForBytesWritten(1000);
        socket->waitForReadyRead(3000);
        qDebug() << "Reading:" << socket->bytesAvailable();
        qDebug() << socket->readAll();
    }
    else{
        qDebug() << "Not connected!";
    }
}

void Socket::disconnect(){
    socket->write("Bye()\n");
    socket->waitForBytesWritten(1000);
    socket->waitForReadyRead(3000);
    qDebug() << "Reading:" << socket->bytesAvailable();
    qDebug() << socket->readAll();
}

void Socket::grip(){
    socket->write("grip()\n");
    socket->waitForBytesWritten(1000);
    socket->waitForReadyRead(3000);
    qDebug() << "Reading:" << socket->bytesAvailable();
    qDebug() << socket->readAll();
    socket->waitForReadyRead(3000);
    qDebug() << "Reading:" << socket->bytesAvailable();
    qDebug() << socket->readAll();
}

void Socket::grip(std::string forceInput){
    char force[2];
    force[0]=forceInput.at(0);
    force[1]=forceInput.at(1);
    socket->write("grip(");
    socket->write(force);
    socket->write(")\n");
    socket->waitForBytesWritten(1000);
    socket->waitForReadyRead(3000);
    qDebug() << "Reading:" << socket->bytesAvailable();
    qDebug() << socket->readAll();
    socket->waitForReadyRead(3000);
    qDebug() << "Reading:" << socket->bytesAvailable();
    qDebug() << socket->readAll();
}

void Socket::release(){
    socket->write("release()\n");
    socket->waitForBytesWritten(1000);
    socket->waitForReadyRead(3000);
    qDebug() << "Reading:" << socket->bytesAvailable();
    qDebug() << socket->readAll();
    socket->waitForReadyRead(3000);
    qDebug() << "Reading:" << socket->bytesAvailable();
    qDebug() << socket->readAll();
}

void Socket::release(std::string releaseDistance){
    char dist[2];
    dist[0]=releaseDistance.at(0);
    dist[1]=releaseDistance.at(1);
    socket->write("release(");
    socket->write(dist);
    socket->write(")\n");
    socket->waitForBytesWritten(1000);
    socket->waitForReadyRead(3000);
    qDebug() << "Reading:" << socket->bytesAvailable();
    qDebug() << socket->readAll();
    socket->waitForReadyRead(3000);
    qDebug() << "Reading:" << socket->bytesAvailable();
    qDebug() << socket->readAll();
}

void Socket::home(){
    socket->write("home()\n");
    socket->waitForBytesWritten(1000);
    socket->waitForReadyRead(3000);
    qDebug() << "Reading:" << socket->bytesAvailable();
    qDebug() << socket->readAll();
    socket->waitForReadyRead(3000);
    qDebug() << "Reading:" << socket->bytesAvailable();
    qDebug() << socket->readAll();
}

void Socket::connectMatlab(){

socket = new QTcpSocket(this);

socket->connectToHost("127.0.0.1", 30002);

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

    /*
    uint8_t* in1 = (uint8_t*)(&input1);
    char* res;


    for (int i = 0; i < sizeof(double); ++i) {
       res[i] = in1[i];
    }
    */

    std::string s = std::to_string(input1);

    const char* c = s.c_str();

    socket->write(c);
    socket->write(",");

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



