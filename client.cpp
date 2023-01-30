// Client side C/C++ program to demonstrate Socket
// programming
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <pthread.h>
#include <unordered_map>

#include "/home/allan/draco/src/draco/compression/encode.h"
#include "/home/allan/draco/src/draco/core/cycle_timer.h"
#include "/home/allan/draco/src/draco/io/file_utils.h"
#include "/home/allan/draco/src/draco/io/mesh_io.h"
// #include "/home/allan/draco/src/draco/io/ply_encoder.h"
#include "/home/allan/draco/src/draco/io/point_cloud_io.h"
#include "/home/allan/draco/src/draco/mesh/mesh.h"

#include "open3d/Open3D.h"
#include "/home/allan/Open3D/cpp/open3d/io/TriangleMeshIO.h"
#include "/home/allan/draco_encode_cpp/rplyCustom.h"
#include "/home/allan/draco_encode_cpp/FilePLYCustom.h"
// #include "/home/allan/draco_encode_cpp/stdio_file_writerCustom.h"
#include "/home/allan/draco_encode_cpp/plyEncoderCustom.h"
// #include "plyEncoderCustom.h"


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

typedef struct
{
	int port;
	int id;
} args_t;

struct PLYReaderState
{
	open3d::utility::CountingProgressReporter *progress_bar;
	open3d::geometry::TriangleMesh *mesh_ptr;
	long vertex_index;
	long vertex_num;
	long normal_index;
	long normal_num;
	long color_index;
	long color_num;
	std::vector<unsigned int> face;
	long face_index;
	long face_num;
};

p_ply ply_open(FILE *pFileBuffer, p_ply_error_cb error_cb, long idata, void *pdata)
{
	FILE *fp = NULL;
	p_ply ply = ply_alloc();
	if (error_cb == NULL)
		error_cb = ply_error_cb;
	if (!ply)
	{
		error_cb(NULL, "Out of memory");
		return NULL;
	}
	ply->idata = idata;
	ply->pdata = pdata;
	ply->io_mode = PLY_READ;
	ply->error_cb = error_cb;
	if (!ply_type_check())
	{
		error_cb(ply, "Incompatible type system");
		free(ply);
		return NULL;
	}
	// assert(name);
	// fp = fopen(name, "rb");
	fp = pFileBuffer;
	if (!fp)
	{
		error_cb(ply, "Unable to open file");
		free(ply);
		return NULL;
	}
	ply->fp = fp;
	printf("ply opened successfully\n");
	return ply;
}

bool ReadTriangleMeshFromPLYCustom(FILE *pFileBuffer, open3d::geometry::TriangleMesh &mesh)
{
	// using namespace open3d;
	// using namespace io;
	// using namespace ply_trianglemesh_reader;
	// p_ply ply_open(const char *name, p_ply_error_cb error_cb, long idata, void *pdata)
	p_ply ply_file = ply_open(pFileBuffer, NULL, 0, NULL);
	if (!ply_file)
	{
		open3d::utility::LogWarning("Read PLY failed: unable to open file BUFFER ");
		return false;
	}
	
	if (!ply_read_header(ply_file))
	{
		open3d::utility::LogWarning("Read PLY failed: unable to parse header.");
		ply_close(ply_file);
		return false;
	}
	printf("DEBUG: in read triangle mesh from ply custom, buffer read\n");
	// ply is not being correctly read here
	PLYReaderState state;
	state.mesh_ptr = &mesh;
	state.vertex_num = ply_set_read_cb(ply_file, "vertex", "x", open3d::io::ply_pointcloud_reader::ReadVertexCallback, &state, 0);
	ply_set_read_cb(ply_file, "vertex", "y", open3d::io::ply_pointcloud_reader::ReadVertexCallback, &state, 1);
	ply_set_read_cb(ply_file, "vertex", "z", open3d::io::ply_pointcloud_reader::ReadVertexCallback, &state, 2);

	state.normal_num = ply_set_read_cb(ply_file, "vertex", "nx", open3d::io::ply_pointcloud_reader::ReadNormalCallback, &state, 0);
	ply_set_read_cb(ply_file, "vertex", "ny", open3d::io::ply_pointcloud_reader::ReadNormalCallback, &state, 1);
	ply_set_read_cb(ply_file, "vertex", "nz", open3d::io::ply_pointcloud_reader::ReadNormalCallback, &state, 2);

	state.color_num = ply_set_read_cb(ply_file, "vertex", "red", open3d::io::ply_pointcloud_reader::ReadColorCallback, &state, 0);
	ply_set_read_cb(ply_file, "vertex", "green", open3d::io::ply_pointcloud_reader::ReadColorCallback, &state, 1);
	ply_set_read_cb(ply_file, "vertex", "blue", open3d::io::ply_pointcloud_reader::ReadColorCallback, &state, 2);

	if (state.vertex_num <= 0)
	{
		open3d::utility::LogWarning("Read PLY failed: number of vertex <= 0.");
		printf("DEBUG: num vertex: %ld\n",state.vertex_num);
		ply_close(ply_file);
		return false;
	}

	state.face_num = ply_set_read_cb(ply_file, "face", "vertex_indices", open3d::io::ply_trianglemesh_reader::ReadFaceCallBack, &state, 0);
	if (state.face_num == 0)
	{
		state.face_num = ply_set_read_cb(ply_file, "face", "vertex_index", open3d::io::ply_trianglemesh_reader::ReadFaceCallBack, &state, 0);
	}

	state.vertex_index = 0;
	state.normal_index = 0;
	state.color_index = 0;
	state.face_index = 0;

	mesh.Clear();
	mesh.vertices_.resize(state.vertex_num);
	mesh.vertex_normals_.resize(state.normal_num);
	mesh.vertex_colors_.resize(state.color_num);

	// open3d::utility::CountingProgressReporter reporter(params.update_progress);
	// reporter.SetTotal(state.vertex_num + state.face_num);
	// state.progress_bar = &reporter;

	if (!ply_read(ply_file))
	{
		open3d::utility::LogWarning("Read PLY failed: unable to read file BUFFER: ");
		ply_close(ply_file);
		return false;
	}

	ply_close(ply_file);
	// reporter.Finish();
	return true;
}

// static const std::unordered_map<
// 	std::string, std::function<bool(const std::string &, open3d::geometry::TriangleMesh &, const open3d::io::ReadTriangleMeshOptions &)>>
// 	file_extension_to_trianglemesh_read_function;

bool ReadTriangleMeshCustom(FILE *pFileBuffer, open3d::geometry::TriangleMesh &mesh)
{
	// std::string filename_ext = open3d::utility::filesystem::GetFileExtensionInLowerCase(filename);
	// if (filename_ext.empty()) {
	//     open3d::utility::LogWarning(
	//             "Read geometry::TriangleMesh failed: unknown file "
	//             "extension.");
	//     return false;
	// }
	// auto map_itr = file_extension_to_trianglemesh_read_function.find(".ply");
	// if (map_itr == file_extension_to_trianglemesh_read_function.end())
	// {
	// 	open3d::utility::LogWarning(
	// 		"Read geometry::TriangleMesh failed: unknown file "
	// 		"extension.");
	// 	return false;
	// }

	// if (params.print_progress) {
	//     auto progress_text = std::string("Reading ") +
	//                          open3d::utility::ToUpper(filename_ext) +
	//                          " file: " + filename;
	//     auto pbar = open3d::utility::ProgressBar(100, progress_text, true);
	//     params.update_progress = [pbar](double percent) mutable -> bool {
	//         pbar.SetCurrentCount(size_t(percent));
	//         return true;
	//     };
	// }

	// save file here for testing
	// FILE* out = fopen("/home/allan/draco_encode_cpp/bun_zipper3.ply","w+");
	

	// bool success = map_itr->second(pFileBuffer, mesh);
	bool success = ReadTriangleMeshFromPLYCustom(pFileBuffer, mesh);
	printf("In read triangle mesh PLY custom: %d\n",success);
	open3d::utility::LogDebug(
		"Read geometry::TriangleMesh: {:d} triangles and {:d} vertices.",
		(int)mesh.triangles_.size(), (int)mesh.vertices_.size());
	if (mesh.HasVertices() && !mesh.HasTriangles())
	{
		open3d::utility::LogWarning(
			"geometry::TriangleMesh appears to be a geometry::PointCloud "
			"(only contains vertices, but no triangles).");
	}
	return success;
}

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

	// doing this is futile -> just need encoder buffer to send to ply encoder
	draco::EncoderBuffer encoderBuffer;
	encoderBuffer.buffer()->insert(encoderBuffer.buffer()->end(), &inBuffer[0], &inBuffer[totalRead]);
	
	draco::DecoderBuffer decoderBuffer;
	decoderBuffer.Init(encoderBuffer.data(), encoderBuffer.size());

	draco::Decoder decoder;
	std::unique_ptr<draco::Mesh> meshToSave = decoder.DecodeMeshFromBuffer(&decoderBuffer).value();
	
	// have draco buffer
	// need to convert in to open3d model
	// want to hijack the outstream and direct the data to a custom out stream instead of a real file

	// fwrite(pFile, sizeof(char), totalRead, pFile);
	// for (int i = 0; i < 100; i++)
	// {
	// 	printf("face: %d %d %d \n", meshToSave->face(draco::FaceIndex(i))[0],
	// 		   meshToSave->face(draco::FaceIndex(i))[1],
	// 		   meshToSave->face(draco::FaceIndex(i))[2]);

	// 	// std::cout << inBuffer[i];
	// 	// std::cout << meshToSave->face(draco::FaceIndex(i))[1].value() << " \n ";
	// }
	
	

	// draco::PlyEncoder plyEncoder;
	// custom plyEncoderCustom;
	char outPath[1024] = {0};
	// sprintf(outPath, "/home/allan/draco_encode_cpp/client/test_thread%d.ply", args->id);
	sprintf(outPath, "/home/allan/draco_encode_cpp/bun_zipper2.ply");
	// bool success = plyEncoder.EncodeToFile(*(meshToSave.get()), outPath);
	// Without lock: Bad file descriptor (corrupts the .ply file)
	
	// need to call encoderInternal() for things to work -> call buffer
	printf("DEBUG: \n");
	FILE *pFile;
	char empty[totalRead * sizeof(char) * 10] = {0};
	// pFile = fmemopen(NULL, totalRead * sizeof(char) * 10, "w+");
	pFile = fmemopen(&empty, totalRead * sizeof(char) * 10, "w+");
	// pthread_mutex_lock(&fileMutex);
	
	// FILE* ptr = fopen("/home/allan/draco_encode_cpp/bun_zipper2.ply","w+");
	// bool success = fwrite(encoderBuffer.data(), 1, encoderBuffer.size(), ptr) == encoderBuffer.size();
	// fclose(ptr);

	// need to read in mesh for it to be properly converted to PLY format
	PlyEncoderCustom plyEncoderCustom;
	// plyEncoderCustom.EncodeToFileBuffer(*(meshToSave.get()), ptr);
	// plyEncoderCustom.PlyEncoder();
	// plyEncoderCustom.EncodeToFileBuffer(*meshToSave.get(),ptr);
	// bool success = plyEncoderCustom.EncodeToFile(*(meshToSave.get()), outPath);
	bool success = plyEncoderCustom.EncodeToFileBuffer(*(meshToSave.get()), pFile);
	printf("DEBUG: \nread start: %s\n read end: %s\n read curr: %s\n",pFile->_IO_read_base, pFile->_IO_read_end, pFile->_IO_read_ptr);
	// printf("DEBUG: \nread start: %s\n read end: %s\n read curr: %s\n",pFile->io, pFile->_IO_read_end, pFile->_IO_read_ptr);
	// fclose(pFile);
	
	
	
	// pthread_mutex_unlock(&fileMutex);
	// pFile = fopen("/home/allan/draco_encode_cpp/bunny.ply","w+");
	// bool success = plyEncoderCustom.EncodeToFileBuffer(*(meshToSave.get()), pFile);
	// bool success = plyEncoderCustom.EncodeToFileBufferTest(*(meshToSave.get()), pFile);
	
	printf("buffer save success: %d\n", success);

	// now use open3d to read this virtual file (in the form of a file pointer stream)

	// display the photo
	if (args->id == 0)
	{
		// pthread_mutex_lock(&fileMutex);
		auto mesh_ptr = std::make_shared<open3d::geometry::TriangleMesh>();
		// if (open3d::io::ReadTriangleMesh("/home/allan/draco_encode_cpp/bun_zipper2.ply", *mesh_ptr))
		if (ReadTriangleMeshCustom(pFile, *mesh_ptr))
		{
			open3d::utility::LogInfo("Successfully read {}\n", "/home/allan/draco_encode_cpp/bun_zipper.ply");
		}
		else
		{
			open3d::utility::LogWarning("Failed to read {}\n\n", "/home/allan/draco_encode_cpp/bun_zipper.ply");
			// return 1;
		}
		mesh_ptr->ComputeVertexNormals();
		open3d::visualization::DrawGeometries({mesh_ptr}, "Mesh", 1600, 900);
		// pthread_mutex_unlock(&fileMutex);
		// utility::LogInfo("vertices: {} {} {}\n",mesh_ptr->vertices_[0][0], mesh_ptr->vertices_[0][1], mesh_ptr->vertices_[0][2]);
		open3d::utility::LogInfo("vertices: {}\n", mesh_ptr->vertices_);
		open3d::utility::LogInfo("triangles: {}\n", mesh_ptr->triangles_);
		
	}
	fclose(pFile);
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
