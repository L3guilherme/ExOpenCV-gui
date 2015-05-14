#ifndef CONNECTR_H
#define CONNECTR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <arpa/inet.h>
#include <netdb.h>


class ConnectR
{
public:
    ConnectR();
    ~ConnectR();
    void InicServ(int port);
    void InicClient(std::string ip, int port);
    void SendMsgToServ(std::string msg);
    void SendMsgToClient(std::string msg);
    void RecieveFromServ();
    void RecieveFromClient();
    void CloseConnectons();
private:
    int sockfdServ, sockfdClient, portServ,portClint,sockfdClientFromServ;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;
    struct hostent *serverFromClient;
    void WaitConnection();
    void RecievingFromServ();
    void RecievingFromClient();


};

#endif // CONNECTR_H
