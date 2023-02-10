// Server side C/C++ program to demonstrate Socket
// programming
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <pthread.h>
#include <netdb.h>

#include "../draco/src/draco/compression/encode.h"
#include "../draco/src/draco/core/cycle_timer.h"
#include "../draco/src/draco/io/file_utils.h"
#include "../draco/src/draco/io/mesh_io.h"
#include "../draco/src/draco/io/point_cloud_io.h"
#include "../draco/src/draco/io/obj_encoder.h"
#include "./utils.h"
#include "open3d/Open3D.h"


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
bool enableDebugging = 0;

typedef struct
{
	int port;
	int id;
} args_t;


int open3d_to_draco(open3d::geometry::TriangleMesh *inOpen3d, draco::EncoderBuffer *meshBuffer){

    draco::Mesh *open3dToDracoMesh = new draco::Mesh();
    open3dToDracoMesh->set_num_points((uint32_t)inOpen3d->vertices_.size());
    open3dToDracoMesh->SetNumFaces(inOpen3d->triangles_.size());
    draco::PointAttribute open3dToDracoPositionAttribute;
    draco::PointAttribute open3dToDracoNormalAttribute;

    // type based off enum in geometry_attribute.h
    open3dToDracoPositionAttribute.Init(draco::PointAttribute::Type(0), 3, draco::DataType::DT_FLOAT64, true, inOpen3d->vertices_.size());
    open3dToDracoNormalAttribute.Init(draco::PointAttribute::Type(1), 3, draco::DataType::DT_FLOAT64, true, inOpen3d->vertex_normals_.size());

    // add these attributes to the mesh
    int32_t open3dToDracoPositionAttributeID = open3dToDracoMesh->AddAttribute(open3dToDracoPositionAttribute, true, (uint32_t)inOpen3d->vertices_.size());
    int32_t open3dToDracoNormalAttributeID = open3dToDracoMesh->AddAttribute(open3dToDracoNormalAttribute, true, (uint32_t)inOpen3d->vertex_normals_.size());

    // initialize attribute vertex values from open3d
    unsigned int inOpen3dAttributeIdx = 0;
    for (auto itr = inOpen3d->vertices_.begin(); itr != inOpen3d->vertices_.end(); itr++)
    {
        if (enableDebugging)
        {
            std::cout << (*itr)[0] << ", " << (*itr)[1] << ", " << (*itr)[2] << "\n";
        }
        double inOpen3dVertex[3];
        inOpen3dVertex[0] = (*itr)[0];
        inOpen3dVertex[1] = (*itr)[1];
        inOpen3dVertex[2] = (*itr)[2];
        open3dToDracoMesh->attribute(open3dToDracoPositionAttributeID)->SetAttributeValue(draco::AttributeValueIndex(inOpen3dAttributeIdx), &inOpen3dVertex[0]);
        inOpen3dAttributeIdx++;
    }

    inOpen3dAttributeIdx = 0;
    for (auto itr = inOpen3d->vertex_normals_.begin(); itr != inOpen3d->vertex_normals_.end(); itr++)
    {
        if (enableDebugging)
        {
            std::cout << (*itr)[0] << ", " << (*itr)[1] << ", " << (*itr)[2] << "\n";
        }
        double inOpen3dVertex[3];
        inOpen3dVertex[0] = (*itr)[0];
        inOpen3dVertex[1] = (*itr)[1];
        inOpen3dVertex[2] = (*itr)[2];
        open3dToDracoMesh->attribute(open3dToDracoNormalAttributeID)->SetAttributeValue(draco::AttributeValueIndex(inOpen3dAttributeIdx), &inOpen3dVertex[0]);
        inOpen3dAttributeIdx++;
    }

    // faces look to be successfully added to the open3dtodracomesh -> failure must be somewhere else TODO
    for (unsigned long i = 0; i < inOpen3d->triangles_.size(); ++i)
    {
        // adding faces broken
        // const draco::Mesh::Face tmpFace({draco::PointIndex((uint32_t)inOpen3d->triangles_[i][0]),draco::PointIndex((uint32_t)inOpen3d->triangles_[i][1]),draco::PointIndex((uint32_t)inOpen3d->triangles_[i][2])});
        draco::Mesh::Face tmpFace = draco::Mesh::Face();
        tmpFace[0] = draco::PointIndex((uint32_t)inOpen3d->triangles_[i][0]);
        tmpFace[1] = draco::PointIndex((uint32_t)inOpen3d->triangles_[i][1]);
        tmpFace[2] = draco::PointIndex((uint32_t)inOpen3d->triangles_[i][2]);

        // face already initialized when face size is set
        open3dToDracoMesh->SetFace(draco::FaceIndex((uint32_t)i), tmpFace);
    }

    // draco::EncoderBuffer meshBuffer;
    // draco::Mesh *meshToSave = nullptr;
    draco::Mesh *mesh = nullptr;

    // draco::StatusOr<std::unique_ptr<draco::Mesh>> maybe_mesh = draco::ReadMeshFromFile("/home/allan/draco_encode_cpp/custom0.obj", false);
    // if (!maybe_mesh.ok())
    // {
    //     printf("Failed loading the input mesh: %s.\n", maybe_mesh.status().error_msg());
    //     throw std::exception();
    // }

    // mesh = maybe_mesh.value().get();
    mesh = open3dToDracoMesh;
    // pc = std::move(maybe_mesh).value();
    // pc = maybe_mesh.value().get();

    // Convert compression level to speed (that 0 = slowest, 10 = fastest).
    const int speed = 10 - 1;
    // const int speed = 1;

    draco::Encoder encoder;
    encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, 10);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 10);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL, 10);
    encoder.SetSpeedOptions(speed, speed);

    // const bool input_is_mesh = mesh && mesh->num_faces() > 0;

    // Convert to ExpertEncoder that allows us to set per-attribute options.
    std::unique_ptr<draco::ExpertEncoder> expert_encoder;
    // if (input_is_mesh)
    // {
    expert_encoder.reset(new draco::ExpertEncoder(*mesh));
    // }
    // else
    // {
    //     expert_encoder.reset(new draco::ExpertEncoder(*pc));
    // }
    // expert_encoder->Reset(encoder.CreateExpertEncoderOptions(*pc));

    // if (input_is_mesh)
    // {
    // std::cout << "about to encode" << std::endl;
    
    
    // encoder.EncodeMeshToBuffer(*mesh, &meshBuffer);
    encoder.EncodeMeshToBuffer(*mesh, meshBuffer);


    // }
    // else
    // {
    //     encoder.EncodePointCloudToBuffer(*pc, &meshBuffer);
    // }
    // std::cout << "finished encoding " << std::endl;
    // draco::DecoderBuffer decoderBuffer;
    // decoderBuffer.Init(meshBuffer.data(), meshBuffer.size());

    // draco::Decoder decoder;

    return 0;
}


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

	// IN:  The char array 'localhostname' 
	// OUT: The hostent struct hp  
	// OUT: A partially filled in sockadd_in struct sa 
	// struct sockadd_in sa; 
	// struct hostent *hp;
	// hp = gethostbyname(localhostname);
	// sa.sin_family = hp->h_addrtype;
	// bcopy((char *)hp->h_addr, (char *)&sa.sin_addr, hp->h_length);

	// address.sin_family = AF_INET;
	struct hostent *hp;
	hp = gethostbyname("192.168.1.23");
	address.sin_family = hp->h_addrtype;
	bcopy((char *)hp->h_addr, (char *)&address.sin_addr, hp->h_length);
	// address.sin_addr.s_addr = INADDR_ANY;
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

	// draco::EncoderBuffer meshBuffer;

	// std::unique_ptr<draco::PointCloud> pc;
	// draco::Mesh *mesh = nullptr;

	// auto maybe_mesh = draco::ReadMeshFromFile("/home/sc/draco_encode_cpp/example2.obj", false);
	// if (!maybe_mesh.ok())
	// {
	// 	printf("Failed loading the input mesh: %s.\n", maybe_mesh.status().error_msg());
	// 	throw std::exception();
	// }

	// mesh = maybe_mesh.value().get();
	// pc = std::move(maybe_mesh).value();

	// // Convert compression level to speed (that 0 = slowest, 10 = fastest).
	// const int speed = 10 - 1;

	// draco::Encoder encoder;
	// encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, 16);
	// encoder.SetSpeedOptions(speed, speed);

	// const bool input_is_mesh = mesh && mesh->num_faces() > 0;

	// // Convert to ExpertEncoder that allows us to set per-attribute options.
	// std::unique_ptr<draco::ExpertEncoder> expert_encoder;
	// if (input_is_mesh)
	// {
	// 	expert_encoder.reset(new draco::ExpertEncoder(*mesh));
	// }
	// else
	// {
	// 	expert_encoder.reset(new draco::ExpertEncoder(*pc));
	// }
	// expert_encoder->Reset(encoder.CreateExpertEncoderOptions(*pc));

	// int ret = -1;

	// if (input_is_mesh)
	// {
	// 	encoder.EncodeMeshToBuffer(*mesh, &meshBuffer);
	// }
	// else
	// {
	// 	encoder.EncodePointCloudToBuffer(*pc, &meshBuffer);
	// }

	auto inOpen3d = std::make_shared<open3d::geometry::TriangleMesh>();
    open3d::io::ReadTriangleMeshOptions inOpt;
    // open3d::io::ReadTriangleMeshFromOBJ("/home/allan/draco_encode_cpp/custom0.obj", *inOpen3d, inOpt);
    open3d::io::ReadTriangleMeshFromOBJ("/home/sc/draco_encode_cpp/example2.obj", *inOpen3d, inOpt);

	draco::EncoderBuffer meshBuffer;

    open3d_to_draco(inOpen3d.get(), &meshBuffer);

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
