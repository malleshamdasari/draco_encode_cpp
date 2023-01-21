// Client side C/C++ program to demonstrate Socket
// programming
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "/home/allan/draco/src/draco/compression/encode.h"
#include "/home/allan/draco/src/draco/core/cycle_timer.h"
#include "/home/allan/draco/src/draco/io/file_utils.h"
#include "/home/allan/draco/src/draco/io/mesh_io.h"
#include "/home/allan/draco/src/draco/io/point_cloud_io.h"

using namespace std;
const char *get_error_text() {

#if defined(_WIN32)

    static char message[256] = {0};
    FormatMessage(
        FORMAT_MESSAGE_FROM_SYSTEM|FORMAT_MESSAGE_IGNORE_INSERTS,
        0, WSAGetLastError(), 0, message, 256, 0);
    char *nl = strrchr(message, '\n');
    if (nl) *nl = 0;
    return message;

#else

    return strerror(errno);

#endif

}

#define MAX 4096
#define PORT 8080

int main(int argc, char const *argv[])
{
	int sock = 0;
	int valread = 0;
	int client_fd = 0;
	struct sockaddr_in serv_addr;
	
	char buffer[1024] = {0};
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n Socket creation error \n");
		return -1;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);

	// Convert IPv4 and IPv6 addresses from text to binary
	// form
	if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}

	if ((client_fd = connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr))) < 0)
	{
		printf("\nConnection Failed \n");
		return -1;
	}

	

	// char* clientInit = "initialize transfer";
	// send(sock, clientInit, strlen(clientInit), 0);
	// printf("client sending: initialize transfer\n");

	// will probably break when the file is too small...

	valread = read(sock, buffer, 1024);
	int toRead = atoi(buffer);
	printf("client toRead: %d\n", toRead);
	printf("read: %d\n", valread);
	
	char inBuffer[toRead] = {0};

	int totalRead = 0;
	while (toRead >= MAX)
	{
		valread = read(sock, (inBuffer + totalRead), MAX);
		totalRead += valread;
		toRead -= valread;
		printf("read: %d toRead: %d\n", valread, toRead);
	}
	
	valread = read(sock, (inBuffer + totalRead), toRead);
	totalRead += valread;
	toRead -= valread;
	printf("read: %d toRead: %d\n", valread, toRead);
	
	draco::EncoderBuffer encoderBuffer;
	encoderBuffer.buffer()->insert(encoderBuffer.buffer()->end(), &inBuffer[0], &inBuffer[totalRead]);

	draco::WriteBufferToFile(encoderBuffer.data(), totalRead, "/home/allan/draco_encode_cpp/client/test.drc");

	// closing the connected socket
	// printf("Last error was: %s\n", get_error_text());
	close(client_fd);
	// printf("Last error was: %s\n", get_error_text());
	return 0;
}
