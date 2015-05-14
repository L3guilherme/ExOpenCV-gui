#include "connectr.hpp"
#include <thread>

ConnectR::ConnectR()
{
}

ConnectR::~ConnectR()
{

}

void ConnectR::InicServ(int port)
{

    // create a socket
    // socket(int domain, int type, int protocol)
    sockfdServ =  socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfdServ >= 0){

        // clear address structure
        bzero((char *) &serv_addr, sizeof(serv_addr));

        portServ = port;

        /* setup the host_addr structure for use in bind call */
        // server byte order
        serv_addr.sin_family = AF_INET;

        // automatically be filled with current host's IP address
        serv_addr.sin_addr.s_addr = INADDR_ANY;

        // convert short integer value for port must be converted into network byte order
        serv_addr.sin_port = htons(portServ);

        // bind(int fd, struct sockaddr *local_addr, socklen_t addr_length)
        // bind() passes file descriptor, the address structure,
        // and the length of the address structure
        // This bind() call will bind  the socket to the current IP address on port, portno
        if (bind(sockfdServ, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) >= 0){

            std::thread t(&ConnectR::WaitConnection,this);
            t.detach();

        }else{
            std::cout<<"ERROR on binding"<<std::endl;
        }


    }else{
        std::cout<<"ERROR opening socket"<<std::endl;
    }


}

void ConnectR::InicClient(std::string ip, int port){
    portClint = port;
    sockfdClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfdClient >= 0){
        serverFromClient = gethostbyname(ip.data());
        if (serverFromClient == NULL) {
            std::cout<<stderr<<"ERROR, no such host"<<std::endl;
        }else{
            bzero((char *) &serv_addr, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            bcopy((char *)serverFromClient->h_addr,
                  (char *)&serv_addr.sin_addr.s_addr,
                  serverFromClient->h_length);
            serv_addr.sin_port = htons(portClint);
            int r = connect(sockfdClient, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
            if (r < 0)
                std::cout<<"ERROR connecting: "<<r<<std::endl;
        }
    }else{
        std::cout<<"ERROR opening socket: "<<sockfdClient<<std::endl;
    }


}

void ConnectR::WaitConnection(){

    // This listen() call tells the socket to listen to the incoming connections.
    // The listen() function places all incoming connection into a backlog queue
    // until accept() call accepts the connection.
    // Here, we set the maximum size for the backlog queue to 5.

    std::cout<<"Aguardando conexão..."<<std::endl;
    listen(sockfdServ,5);

    // The accept() call actually accepts an incoming connection
    clilen = sizeof(cli_addr);

    // This accept() function will write the connecting client's address info
    // into the the address structure and the size of that structure is clilen.
    // The accept() returns a new socket file descriptor for the accepted connection.
    // So, the original socket file descriptor can continue to be used
    // for accepting new connections while the new socker file descriptor is used for
    // communicating with the connected client.
    sockfdClientFromServ = accept(sockfdServ, (struct sockaddr *) &cli_addr, &clilen);
    if (sockfdClientFromServ > 0){

        std::cout<<"Server: got connection from"<<inet_ntoa(cli_addr.sin_addr)<<" port: "<<ntohs(cli_addr.sin_port)<<std::endl;

    }else{
        std::cout<<"ERROR on accept"<<std::endl;
    }
}

void ConnectR::SendMsgToServ(std::string msg){
    write(sockfdClient, msg.data(), strlen(msg.data()));
}

void ConnectR::SendMsgToClient(std::string msg){
    write(sockfdClientFromServ, msg.data(), strlen(msg.data()));
}

void ConnectR::CloseConnectons(){
    close(sockfdClient);
    close(sockfdClientFromServ);
    close(sockfdServ);
    std::cout<<"Conexão fechada!"<<std::endl;
}

void ConnectR::RecievingFromClient(){
    char buffer[256];
     std::cout<<"Aguardando msg From Client..."<<std::endl;
    while(true){
        bzero(buffer,256);
        int n = read(sockfdClientFromServ, buffer, 255);
        if(n>0){
            std::cout<<"Mensagem: "<<buffer<<std::endl;
        }
    }
}

void ConnectR::RecievingFromServ(){
    char buffer[256];
     std::cout<<"Aguardando msg from Serv..."<<std::endl;
    while(true){
        bzero(buffer,256);
        int n = read(sockfdClient, buffer, 255);
        if(n>0){
            std::cout<<"Mensagem: "<<buffer<<std::endl;
        }
    }
}

void ConnectR::RecieveFromClient(){
    std::thread t(&ConnectR::RecievingFromClient,this);
    t.detach();
}

void ConnectR::RecieveFromServ(){
    std::thread t(&ConnectR::RecievingFromServ,this);
    t.detach();
}

