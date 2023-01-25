// Client side C/C++ program to demonstrate Socket
// programming
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <pthread.h>

#include "/home/allan/draco/src/draco/compression/encode.h"
#include "/home/allan/draco/src/draco/core/cycle_timer.h"
#include "/home/allan/draco/src/draco/io/file_utils.h"
#include "/home/allan/draco/src/draco/io/mesh_io.h"
#include "/home/allan/draco/src/draco/io/ply_encoder.h"
#include "/home/allan/draco/src/draco/io/point_cloud_io.h"
#include "/home/allan/draco/src/draco/mesh/mesh.h"

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
#define NUM_THREADS 2

int sock = 0;
int valread = 0;
int client_fd = 0;

typedef struct
{
	int port;
	int id;
} args_t;

static void *recieve(void *data)
{
	args_t *args = (args_t *)data;

	int sock = 0;
	int valread = 0;
	int client_fd = 0;
	struct sockaddr_in serv_addr;

	char buffer[1024] = {0};

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n Socket creation error \n");
		// return -1;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(args->port);

	// Convert IPv4 and IPv6 addresses from text to binary
	// form
	if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return NULL;
	}

	if ((client_fd = connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr))) < 0)
	{
		printf("\nConnection Failed in thread: %d\n", args->id);
		return NULL;
	}

	// will probably break when the file is too small...
	valread = read(sock, buffer, 1024);
	int toRead = atoi(buffer);
	printf("(%d) client toRead: %d\n", args->id, toRead);
	printf("(%d) read: %d\n", args->id, valread);

	char inBuffer[toRead] = {0};

	int totalRead = 0;
	while (toRead >= MAX)
	{
		valread = read(sock, (inBuffer + totalRead), MAX);
		totalRead += valread;
		toRead -= valread;
		printf("(%d) read: %d toRead: %d\n", args->id, valread, toRead);
	}

	valread = read(sock, (inBuffer + totalRead), toRead);
	totalRead += valread;
	toRead -= valread;
	printf("(%d) read: %d toRead: %d\n", args->id, valread, toRead);

	draco::EncoderBuffer encoderBuffer;
	encoderBuffer.buffer()->insert(encoderBuffer.buffer()->end(), &inBuffer[0], &inBuffer[totalRead]);

	// char outPath[1024] = {0};
	// sprintf(outPath,"/home/allan/draco_encode_cpp/client/%d/test.drc",args->id);
	// draco::WriteBufferToFile(encoderBuffer.data(), totalRead, outPath);
	// printf("(%d) saved to file: %s\n", args->id, outPath);

	draco::DecoderBuffer decoderBuffer;
	decoderBuffer.Init(encoderBuffer.data(), encoderBuffer.size());

	// draco::Mesh *mesh = nullptr;
	// std::unique_ptr<draco::Mesh> mesh(new Mesh());

	draco::Decoder decoder_no_skip;
	draco::Decoder decoder;
	std::unique_ptr<draco::Mesh> meshToSave = decoder.DecodeMeshFromBuffer(&decoderBuffer).value();
	// decoder.DecodeBufferToGeometry(decoderBuffer, mesh);
	// std::unique_ptr<draco::Mesh> mesh = decoder.DecodeMeshFromBuffer(&decoderBuffer).value();
	// decoder.DecodeBufferToGeometry(decoderBuffer, mesh);
	// mesh = decoder.DecodeMeshFromBuffer(&decoderBuffer);

	char outPath[1024] = {0};
	sprintf(outPath, "/home/allan/draco_encode_cpp/client/%d/test.ply", args->id);
	// draco::WriteBufferToFile(decoder_no_skip, totalRead, outPath);
	// bool EncodeToFile(const Mesh &mesh, const std::string &file_name);
	draco::PlyEncoder plyEncoder;
	plyEncoder.EncodeToFile(*(meshToSave.get()), outPath);
	printf("(%d) saved to file: %s\n", args->id, outPath);

	// closing the connected socket
	// printf("Last error was: %s\n", get_error_text());
	close(client_fd);
	// printf("Last error was: %s\n", get_error_text());
}

int main(int argc, char const *argv[])
{

	pthread_t threads[NUM_THREADS];
	args_t args[NUM_THREADS];

	for (int i = 0; i < NUM_THREADS; i++)
	{
		args[i].port = PORT + i;
		args[i].id = i;
		pthread_create(&threads[i], NULL, recieve, &args[i]);
	}

	for (unsigned i = 0; i < NUM_THREADS; i++)
	{
		pthread_join(threads[i], NULL);
	}

	return 0;
}
