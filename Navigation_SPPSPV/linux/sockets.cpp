#include "GPS_Structs.h"

bool OpenSocket(SocketType& sock, const char IP[], unsigned short Port)
{
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == INVALID_SOCKET)
    {
        return false;
    }

    sockaddr_in addrSrv;
    memset(&addrSrv, 0, sizeof(addrSrv));
    addrSrv.sin_family = AF_INET;
    addrSrv.sin_port = htons(Port);

    if (inet_pton(AF_INET, IP, &addrSrv.sin_addr) <= 0)
    {
        CloseSocket(sock);
        return false;
    }

    if (connect(sock, (sockaddr*)&addrSrv, sizeof(addrSrv)) == SOCKET_ERROR)
    {
        CloseSocket(sock);
        return false;
    }

    return true;
}

void CloseSocket(SocketType& sock)
{
    if (sock != INVALID_SOCKET)
    {
#ifdef _WIN32
        closesocket(sock);
        WSACleanup();
#else
        close(sock);
#endif
        sock = INVALID_SOCKET;
    }
}