// Client side C/C++ program to demonstrate Socket
// programming
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <pthread.h>
#include <unordered_map>
#include <netdb.h>

#include "../draco/src/draco/compression/encode.h"
#include "../draco/src/draco/core/cycle_timer.h"
#include "../draco/src/draco/io/file_utils.h"
#include "../draco/src/draco/io/mesh_io.h"
#include "../draco/src/draco/io/obj_encoder.h"
#include "../draco/src/draco/io/point_cloud_io.h"
#include "../draco/src/draco/mesh/mesh.h"

#include "open3d/Open3D.h"


// #include "open3d/io/FileFormatIO.h"
// #include "open3d/io/LineSetIO.h"
// #include "open3d/io/PointCloudIO.h"
// #include "open3d/io/TriangleMeshIO.h"
// #include "open3d/io/VoxelGridIO.h"
// #include "open3d/utility/Logging.h"
// #include "open3d/utility/ProgressBar.h"
// #include "open3d/utility/ProgressReporters.h"

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

int sock = 0;
int valread = 0;
int client_fd = 0;
pthread_mutex_t fileMutex;
char localhostname[255] = "sc-1.arena.andrew.cmu.edu";

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
	struct sockaddr_in address;

	char buffer[1024] = {0};

	
	struct hostent *hp;
	hp = gethostbyname("192.168.1.79");
	std::cout << hp->h_addr << std::endl;
	address.sin_family = hp->h_addrtype;
	bcopy((char *)hp->h_addr, (char *)&address.sin_addr, hp->h_length);
	address.sin_port = htons(args->port);

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n Socket creation error \n");
		// return -1;
	}

	// address.sin_family = AF_INET;
	


	// Convert IPv4 and IPv6 addresses from text to binary
	// form
	if (inet_pton(AF_INET, "192.168.1.79", &address.sin_addr) <= 0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return NULL;
	}

	if ((client_fd = connect(sock, (struct sockaddr *)&address, sizeof(address))) < 0)
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

	// doing this is futile -> just need encoder buffer to send to ply encoder
	draco::EncoderBuffer encoderBuffer;
	encoderBuffer.buffer()->insert(encoderBuffer.buffer()->end(), &inBuffer[0], &inBuffer[totalRead]);
	
	draco::DecoderBuffer decoderBuffer;
	decoderBuffer.Init(encoderBuffer.data(), encoderBuffer.size());

	draco::Decoder decoder;
	std::unique_ptr<draco::Mesh> meshToSave = decoder.DecodeMeshFromBuffer(&decoderBuffer).value();
	
	draco::ObjEncoder objEncoder;
	char outPath[1024] = {0};
	// sprintf(outPath, "/home/allan/draco_encode_cpp/client/test_thread%d.ply", args->id);
	sprintf(outPath, "/home/sc/draco_encode_cpp/example2_.obj");
	bool success = objEncoder.EncodeToFile(*(meshToSave.get()), outPath);
	// Without lock: Bad file descriptor (corrupts the .ply file)
	
	// pthread_mutex_lock(&fileMutex);
	
	// pthread_mutex_unlock(&fileMutex);
	printf("buffer save success: %d\n", success);

	// now use open3d to read this virtual file (in the form of a file pointer stream)

	// display the photo
	
	
	// pthread_mutex_lock(&fileMutex);
	// plyEncoder.EncodeToFile(*(meshToSave.get()), outPath);
	// pthread_mutex_unlock(&fileMutex);
	// printf("(%d) saved to file: %s\n", args->id, outPath);

	// open3d::utility::LogInfo("vertices: {}\n",inBuffer[0]);

	// closing the connected socket
	printf("Last error was: %s\n", get_error_text());
	close(client_fd);
	return NULL;
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
