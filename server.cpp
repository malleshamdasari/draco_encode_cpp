// Server side C/C++ program to demonstrate Socket
// programming
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
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
	int server_fd, new_socket, valread;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[1024] = {0};

	// Creating socket file descriptor
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	// Forcefully attaching socket to the port 8080
	if (setsockopt(server_fd, SOL_SOCKET,
				   SO_REUSEADDR | SO_REUSEPORT, &opt,
				   sizeof(opt)))
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);

	// Forcefully attaching socket to the port 8080
	if (bind(server_fd, (struct sockaddr *)&address,
			 sizeof(address)) < 0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	if (listen(server_fd, 3) < 0)
	{
		perror("listen");
		exit(EXIT_FAILURE);
	}
	if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
							 (socklen_t *)&addrlen)) < 0)
	{
		perror("accept");
		exit(EXIT_FAILURE);
	}

	// connection established, start transmitting

	// char *serverInit = "transfer started";
	// valread = read(new_socket, buffer, 1024);
	// printf("server: %s\n", buffer);

	draco::EncoderBuffer meshBuffer;
	
	std::unique_ptr<draco::PointCloud> pc;
	draco::Mesh *mesh = nullptr;

	auto maybe_mesh = draco::ReadMeshFromFile("/home/allan/draco_encode_cpp/bun_zipper.ply", false);
	if (!maybe_mesh.ok())
	{
		printf("Failed loading the input mesh: %s.\n", maybe_mesh.status().error_msg());
		throw std::exception();
	}

	mesh = maybe_mesh.value().get();
	pc = std::move(maybe_mesh).value();

	// Convert compression level to speed (that 0 = slowest, 10 = fastest).
	const int speed = 10 - 1;

	draco::Encoder encoder;
	encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, 16);
	encoder.SetSpeedOptions(speed, speed);

	const bool input_is_mesh = mesh && mesh->num_faces() > 0;

	// Convert to ExpertEncoder that allows us to set per-attribute options.
	std::unique_ptr<draco::ExpertEncoder> expert_encoder;
	if (input_is_mesh)
	{
		expert_encoder.reset(new draco::ExpertEncoder(*mesh));
	}
	else
	{
		expert_encoder.reset(new draco::ExpertEncoder(*pc));
	}
	expert_encoder->Reset(encoder.CreateExpertEncoderOptions(*pc));

	int ret = -1;

	if (input_is_mesh)
	{
		encoder.EncodeMeshToBuffer(*mesh, &meshBuffer);
	}
	else
	{
		encoder.EncodePointCloudToBuffer(*pc, &meshBuffer);
	}

	// draco::WriteBufferToFile(meshBuffer.data(), 435464, "/home/allan/draco_encode_cpp/client/test.drc");
	printf("mesh buffer size: %d\n",meshBuffer.size());
	sprintf(buffer,"%d",meshBuffer.size());
	int retSend;
	retSend = send(new_socket, buffer, 1024, 0);
	printf("send: %d\n",retSend);

	char outBuffer[meshBuffer.size()] = {0};
	
	copy(meshBuffer.buffer()->begin(), meshBuffer.buffer()->end(), outBuffer);

	int seek = 0;
	int toTransfer = meshBuffer.size();
	
	while(toTransfer >= MAX){
		
		retSend = send(new_socket, outBuffer + seek, MAX , 0);
		toTransfer -= MAX;
		seek += MAX;
		printf("send: %d\n",retSend);
		printf("Last error was: %s\n", get_error_text());
	}
	retSend = send(new_socket, outBuffer + seek, toTransfer , 0);
	printf("send: %d\n",retSend);
	printf("server: transfer started; return: %d\n",retSend);

	// closing the connected socket
	close(new_socket);
	// closing the listening socket
	shutdown(server_fd, SHUT_RDWR);
	return 0;
}
