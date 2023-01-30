// Server side C/C++ program to demonstrate Socket
// programming
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <pthread.h>

#include "/home/allan/draco/src/draco/compression/encode.h"
#include "/home/allan/draco/src/draco/core/cycle_timer.h"
#include "/home/allan/draco/src/draco/io/file_utils.h"
#include "/home/allan/draco/src/draco/io/mesh_io.h"
#include "/home/allan/draco/src/draco/io/point_cloud_io.h"
// #include "open3d/Open3D.h"

using namespace std;
const char *get_error_text()
{

#if defined(_WIN32)

	static char message[256] = {0};
	FormatMessage(
		FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		0, WSAGetLastError(), 0, message, 256, 0);
	char *nl = strrchr(message, '\n');
	if (nl)
		*nl = 0;
	return message;

#else

	return strerror(errno);

#endif
}

#define MAX 4096
#define PORT 8080
#define NUM_THREADS 1

typedef struct
{
	int port;
	int id;
} args_t;

static void *transfer(void *data)
{
	args_t *args = (args_t *)data;

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
	address.sin_port = htons(args->port);

	// Forcefully attaching socket to the port 8080
	if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	if (listen(server_fd, 3) < 0)
	{
		perror("listen");
		exit(EXIT_FAILURE);
	}
	if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
	{
		perror("accept");
		exit(EXIT_FAILURE);
	}

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

	printf("(%d) mesh buffer size: %ld\n", args->id, meshBuffer.size());
	sprintf(buffer, "%ld", meshBuffer.size());
	int response;
	printf("(%d) server: transfer started; return: %d\n", args->id, response);
	response = send(new_socket, buffer, 1024, 0);
	printf("(%d) send: %d\n", args->id, response);

	// connection established, start transmitting
	char outBuffer[meshBuffer.size()] = {0};
	copy(meshBuffer.buffer()->begin(), meshBuffer.buffer()->end(), outBuffer);

	int seek = 0;
	int toTransfer = meshBuffer.size();

	while (toTransfer >= MAX)
	{
		response = send(new_socket, outBuffer + seek, MAX, 0);
		toTransfer -= MAX;
		seek += MAX;
		printf("(%d) send: %d\n", args->id, response);
		// printf("(%d) Last error was: %s\n", args->id, get_error_text());
	}
	response = send(new_socket, outBuffer + seek, toTransfer, 0);
	printf("(%d) send: %d\n", args->id, response);
	
	// closing the connected socket
	close(new_socket);
	// closing the listening socket
	shutdown(server_fd, SHUT_RDWR);
	return NULL;
}

int main(int argc, char const *argv[])
{

	pthread_t threads[NUM_THREADS];
	args_t args[NUM_THREADS];

	for (int i = 0; i < NUM_THREADS; i++)
	{
		args[i].port = PORT + i;
		args[i].id = i;
		pthread_create(&threads[i], NULL, transfer, &args[i]);
	}

	for (unsigned i = 0; i < NUM_THREADS; i++)
	{
		pthread_join(threads[i], NULL);
	}

	return 0;
}
