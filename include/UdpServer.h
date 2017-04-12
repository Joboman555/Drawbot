#ifndef UDPSERVER_H
#define UDPSERVER_H
#include <netinet/in.h>

class UdpServer
{
    private:
        int m_sock;
        struct sockaddr_in m_myaddr;
    public:
        UdpServer(char* toAddress, int port);
        virtual ~UdpServer();
        int setUp();
        void sendMessage(char message[]);
        void closeClient();
};

#endif // UDPSERVER_H