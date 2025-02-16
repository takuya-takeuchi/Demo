#include <iostream>
#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#define BUFFER_SIZE 1024

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Demo <ip> <port>" << std::endl;
        return -1;
    }

    // prepare socket struct
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));

    // check parameter
    int result = inet_pton(AF_INET, argv[1], &servaddr.sin_addr);
    if (result == 0)
    {
        std::cerr << "Invalid IP address format." << std::endl;
        return 1;
    }

    if (result < 0)
    {
        std::cerr << "Error occurred during the IP address conversion." << std::endl;
        return 1;
    }

    char *endptr;
    const long port = std::strtol(argv[2], &endptr, 10);
    if (*endptr != '\0' || !(1 <= port && port <= 65535))
    {
        std::cerr << "port number shall be 1 - 65535" << std::endl;
        return -1;
    }

    // open socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        std::cerr << "Error opening socket" << std::endl;
        return 1;
    }

    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);

    // bind socket
    if (bind(sockfd, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0)
    {
        std::cerr << "Error on binding" << std::endl;
        return 1;
    }

    // receive data
    char buffer[BUFFER_SIZE];
    while (true)
    {
        int n = recv(sockfd, buffer, BUFFER_SIZE, 0);
        if (n < 0)
        {
            std::cerr << "Error in recv" << std::endl;
            break;
        }

        buffer[n] = '\0'; // 終端文字を追加
        std::cout << "Received: " << buffer << std::endl;
    }

    // close socket
    close(sockfd);

    return 0;
}