#ifndef CONNECTR_H
#define CONNECTR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>


class ConnectR
{
public:
    ConnectR();
    ~ConnectR();
    void InicServ();
};

#endif // CONNECTR_H
