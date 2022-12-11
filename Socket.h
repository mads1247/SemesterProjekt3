#ifndef SOCKET_H
#define SOCKET_H

#include <QObject>
#include <QTcpSocket>
#include <QDebug>
#include <QCoreApplication>
#include <string>
#include <sstream>

class Socket : public QObject
{
    Q_OBJECT
public:
    explicit Socket(QObject *parent = nullptr);

    void connect();
    void connectMatlab();

    void writeMatlab(double input1);

    std::string readMatlab();

    void disconnectMatlab();



private:
    QTcpSocket *socket;

    //port nummer til robotten
    int portNR = 30000;



};

#endif // SOCKET_H
