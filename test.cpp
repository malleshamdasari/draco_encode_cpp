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
#include "/home/allan/eigen/Eigen/Core"

#include "/home/allan/Open3D/cpp/open3d/io/TriangleMeshIO.h"
// #include "/home/allan/draco_encode_cpp/rplyCustom.h"
// #include "/home/allan/draco_encode_cpp/rplyCustom.cpp"
// #include "/home/allan/draco_encode_cpp/FilePLYCustom.h"
// #include "/home/allan/draco_encode_cpp/stdio_file_writerCustom.h"
#include "/home/allan/draco_encode_cpp/plyEncoderCustom.h"
#include "/home/allan/draco/src/draco/io/ply_encoder.h"

// #include "open3d/io/FileFormatIO.h"
// #include "open3d/io/LineSetIO.h"
// #include "open3d/io/PointCloudIO.h"
// #include "open3d/io/TriangleMeshIO.h"
// #include "open3d/io/VoxelGridIO.h"
// #include "open3d/utility/Logging.h"
// #include "open3d/utility/ProgressBar.h"
// #include "open3d/utility/ProgressReporters.h"

using namespace std;

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

// p_ply ply_open(FILE *pFileBuffer, p_ply_error_cb error_cb, long idata, void *pdata)
// {
// 	FILE *fp = NULL;
// 	p_ply ply = ply_alloc();
// 	if (error_cb == NULL)
// 		error_cb = ply_error_cb;
// 	if (!ply)
// 	{
// 		error_cb(NULL, "Out of memory");
// 		return NULL;
// 	}
// 	ply->idata = idata;
// 	ply->pdata = pdata;
// 	ply->io_mode = PLY_READ;
// 	ply->error_cb = error_cb;
// 	if (!ply_type_check())
// 	{
// 		error_cb(ply, "Incompatible type system");
// 		free(ply);
// 		return NULL;
// 	}
// 	// assert(name);
// 	// fp = fopen(name, "rb");
// 	fp = pFileBuffer;
// 	if (!fp)
// 	{
// 		error_cb(ply, "Unable to open file");
// 		free(ply);
// 		return NULL;
// 	}
// 	ply->fp = fp;
// 	printf("ply opened successfully\n");
// 	return ply;
// }

// bool ReadTriangleMeshFromPLYCustom(FILE *pFileBuffer, open3d::geometry::TriangleMesh &mesh)
// {
// 	// using namespace open3d;
// 	// using namespace io;
// 	// using namespace ply_trianglemesh_reader;
// 	// p_ply ply_open(const char *name, p_ply_error_cb error_cb, long idata, void *pdata)
// 	p_ply ply_file = ply_open(pFileBuffer, NULL, 0, NULL);
// 	if (!ply_file)
// 	{
// 		open3d::utility::LogWarning("Read PLY failed: unable to open file BUFFER ");
// 		return false;
// 	}

// 	if (!ply_read_header(ply_file))
// 	{
// 		open3d::utility::LogWarning("Read PLY failed: unable to parse header.");
// 		ply_close(ply_file);
// 		return false;
// 	}
// 	printf("DEBUG: in read triangle mesh from ply custom, buffer read\n");
// 	// ply is not being correctly read here
// 	PLYReaderState state;
// 	state.mesh_ptr = &mesh;
// 	state.vertex_num = ply_set_read_cb(ply_file, "vertex", "x", open3d::io::ply_pointcloud_reader::ReadVertexCallback, &state, 0);
// 	ply_set_read_cb(ply_file, "vertex", "y", open3d::io::ply_pointcloud_reader::ReadVertexCallback, &state, 1);
// 	ply_set_read_cb(ply_file, "vertex", "z", open3d::io::ply_pointcloud_reader::ReadVertexCallback, &state, 2);

//     printf("read first vertex\n");

// 	state.normal_num = ply_set_read_cb(ply_file, "vertex", "nx", open3d::io::ply_pointcloud_reader::ReadNormalCallback, &state, 0);
// 	ply_set_read_cb(ply_file, "vertex", "ny", open3d::io::ply_pointcloud_reader::ReadNormalCallback, &state, 1);
// 	ply_set_read_cb(ply_file, "vertex", "nz", open3d::io::ply_pointcloud_reader::ReadNormalCallback, &state, 2);

//     printf("read second vertex\n");

// 	state.color_num = ply_set_read_cb(ply_file, "vertex", "red", open3d::io::ply_pointcloud_reader::ReadColorCallback, &state, 0);
// 	ply_set_read_cb(ply_file, "vertex", "green", open3d::io::ply_pointcloud_reader::ReadColorCallback, &state, 1);
// 	ply_set_read_cb(ply_file, "vertex", "blue", open3d::io::ply_pointcloud_reader::ReadColorCallback, &state, 2);

//     printf("read third vertex\n");

// 	if (state.vertex_num <= 0)
// 	{
// 		open3d::utility::LogWarning("Read PLY failed: number of vertex <= 0.");
// 		printf("DEBUG: num vertex: %ld\n",state.vertex_num);
// 		ply_close(ply_file);
// 		return false;
// 	}

//     printf("passed vertex check\n");

// 	state.face_num = ply_set_read_cb(ply_file, "face", "vertex_indices", open3d::io::ply_trianglemesh_reader::ReadFaceCallBack, &state, 0);
// 	if (state.face_num == 0)
// 	{
// 		state.face_num = ply_set_read_cb(ply_file, "face", "vertex_index", open3d::io::ply_trianglemesh_reader::ReadFaceCallBack, &state, 0);
// 	}

//     printf("passed face check\n");

// 	state.vertex_index = 0;
// 	state.normal_index = 0;
// 	state.color_index = 0;
// 	state.face_index = 0;

// 	mesh.Clear();
// 	mesh.vertices_.resize(state.vertex_num);
// 	mesh.vertex_normals_.resize(state.normal_num);
// 	mesh.vertex_colors_.resize(state.color_num);

// 	// open3d::utility::CountingProgressReporter reporter(params.update_progress);
// 	// reporter.SetTotal(state.vertex_num + state.face_num);
// 	// state.progress_bar = &reporter;
//     printf("before ply_read\n");
// 	if (!ply_read(ply_file))
// 	{
//         printf("in ply_read if statement\n");
// 		open3d::utility::LogWarning("Read PLY failed: unable to read file BUFFER: ");
// 		ply_close(ply_file);
// 		return false;
// 	}

//     printf("about to close ply \n");

// 	ply_close(ply_file);
// 	// reporter.Finish();
// 	return true;
// }

// // static const std::unordered_map<
// // 	std::string, std::function<bool(const std::string &, open3d::geometry::TriangleMesh &, const open3d::io::ReadTriangleMeshOptions &)>>
// // 	file_extension_to_trianglemesh_read_function;

// bool ReadTriangleMeshCustom(FILE *pFileBuffer, open3d::geometry::TriangleMesh &mesh)
// {
// 	// std::string filename_ext = open3d::utility::filesystem::GetFileExtensionInLowerCase(filename);
// 	// if (filename_ext.empty()) {
// 	//     open3d::utility::LogWarning(
// 	//             "Read geometry::TriangleMesh failed: unknown file "
// 	//             "extension.");
// 	//     return false;
// 	// }
// 	// auto map_itr = file_extension_to_trianglemesh_read_function.find(".ply");
// 	// if (map_itr == file_extension_to_trianglemesh_read_function.end())
// 	// {
// 	// 	open3d::utility::LogWarning(
// 	// 		"Read geometry::TriangleMesh failed: unknown file "
// 	// 		"extension.");
// 	// 	return false;
// 	// }

// 	// if (params.print_progress) {
// 	//     auto progress_text = std::string("Reading ") +
// 	//                          open3d::utility::ToUpper(filename_ext) +
// 	//                          " file: " + filename;
// 	//     auto pbar = open3d::utility::ProgressBar(100, progress_text, true);
// 	//     params.update_progress = [pbar](double percent) mutable -> bool {
// 	//         pbar.SetCurrentCount(size_t(percent));
// 	//         return true;
// 	//     };
// 	// }

// 	// save file here for testing
// 	// FILE* out = fopen("/home/allan/draco_encode_cpp/bun_zipper3.ply","w+");

// 	// bool success = map_itr->second(pFileBuffer, mesh);
// 	bool success = ReadTriangleMeshFromPLYCustom(pFileBuffer, mesh);
// 	printf("In read triangle mesh PLY custom: %d\n",success);
// 	open3d::utility::LogDebug(
// 		"Read geometry::TriangleMesh: {:d} triangles and {:d} vertices.",
// 		(int)mesh.triangles_.size(), (int)mesh.vertices_.size());
// 	if (mesh.HasVertices() && !mesh.HasTriangles())
// 	{
// 		open3d::utility::LogWarning(
// 			"geometry::TriangleMesh appears to be a geometry::PointCloud "
// 			"(only contains vertices, but no triangles).");
// 	}
// 	return success;
// }

int main(int argc, char const *argv[])
{

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

    draco::DecoderBuffer decoderBuffer;
    decoderBuffer.Init(meshBuffer.data(), meshBuffer.size());

    draco::Decoder decoder;
    std::unique_ptr<draco::Mesh> meshToSave = decoder.DecodeMeshFromBuffer(&decoderBuffer).value();

    char outPath[1024] = {0};
    // sprintf(outPath, "/home/allan/draco_encode_cpp/client/test_thread%d.ply", args->id);
    sprintf(outPath, "/home/allan/draco_encode_cpp/bun_zipperDraco.ply");

    // file read into draco
    // PlyEncoderCustom plyEncoderCustom;
    // draco::PlyEncoder plyEncoder;
    // FILE *pFile;
    // char empty[meshBuffer.size() * sizeof(char) * 10] = {0};
    // pFile = fmemopen(NULL, totalRead * sizeof(char) * 10, "w+");
    // int size = meshBuffer.size() * sizeof(char) * 2;
    // pFile = fmemopen(NULL,size , "w+");
    // bool success = plyEncoderCustom.EncodeToFileBuffer(*meshToSave.get(), pFile);
    // bool success = plyEncoder.EncodeToFile(*meshToSave, outPath);
    // FILE* pFile2 = fopen("/home/allan/draco_encode_cpp/bun_zipper5.ply", "w+");

    printf("mesh num attributes: %d\n", meshToSave->num_attributes());
    const draco::PointAttribute *attr = meshToSave->GetAttributeByUniqueId(0);
    cout << attr->TypeToString(attr->attribute_type()) << endl;

    cout << "mesh attribute: " << meshToSave->GetAttributeElementType(0) << std::endl;
    cout << "mesh number of faces: " << meshToSave->num_faces() << std::endl;
    cout << "mesh number of points: " << meshToSave->num_points() << std::endl;
    // cout << "mesh number of points: " << meshToSave->face() << std::endl;

    // draco triangles
    std::cout << "draco triangles: " << std::endl;
    for (int i = 0; (i < meshToSave->num_faces()) && (i < 10); i++)
    {
        std::cout << (meshToSave->face((draco::FaceIndex)i).begin())->value() << std::endl;
        std::cout << (meshToSave->face((draco::FaceIndex)i).begin() + 1)->value() << std::endl;
        std::cout << (meshToSave->face((draco::FaceIndex)i).begin() + 2)->value() << std::endl;
        std::cout << std::endl;
    }

    // std::cout << "attr map size: " << attr->indices_map_size() << std::endl;
    // int outData[10]={0};
    // std::array<T, att_components_t>
    // std::array<int, 3> outData;
    // draco vertices
    std::cout << "draco vertices: " << std::endl;
    const std::unique_ptr<float[]> att_val(new float[3]);
    for (int i = 0; (i < attr->indices_map_size()) && (i < 10); i++)
    {
        attr->GetValue((draco::AttributeValueIndex)i, att_val.get());
        std::cout << att_val.get()[0] << " " << att_val.get()[1] << " " << att_val.get()[2] << std::endl;
        std::cout << std::endl;
    }

    // char outBuffer[size] = {0};
    // fread(outBuffer, size, 1 , pFile);
    // for(int i = 0; i < size; i++){
    //     std::cout << outBuffer[i] << std::endl;
    // }
    // fwrite(outBuffer, size, 1, pFile2);
    // fclose(pFile2);

    // bool success = plyEncoder.EncodeToFile(*mesh, outPath);

    auto mesh_ptr = std::make_shared<open3d::geometry::TriangleMesh>();
    if (open3d::io::ReadTriangleMesh("/home/allan/draco_encode_cpp/bun_zipper.ply", *mesh_ptr))
    {
        open3d::utility::LogInfo("Successfully read {}\n", "/home/allan/draco_encode_cpp/bun_zipper.ply");
    }
    else
    {
        open3d::utility::LogWarning("Failed to read {}\n\n", "/home/allan/draco_encode_cpp/bun_zipper.ply");
        // return 1;
    }
    // (open3d::io::WriteTriangleMesh("/home/allan/draco_encode_cpp/bun_zipperOpen3d.ply", *mesh_ptr));

    // https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << "mesh triangles dimension: " << mesh_ptr->triangles_.begin().base()->NumDimensions << std::endl;

    std::cout << "open3d vertices: " << std::endl
              << mesh_ptr->vertices_.begin().base()->format(CleanFmt) << std::endl;
    // std::cout << mesh_ptr->vertices_.begin().base()->x() << std::endl;

    std::cout << "open3d triangles: " << std::endl;
    std::vector<Eigen::Vector3i>::iterator meshTmp = mesh_ptr->triangles_.begin();
    int j = 0;
    while (meshTmp != mesh_ptr->triangles_.end())
    {
        std::cout << std::endl;
        std::cout << meshTmp.base()->format(CleanFmt) << std::endl;
        meshTmp++;
        if (j > 10)
        {
            break;
        }
        j++;
    }

    // draco to open3d
    auto open3dOutMesh = std::make_shared<open3d::geometry::TriangleMesh>();
    // open3d::geometry::TriangleMesh open3dOutMesh;
    // write vertices first

    // draco loop
    const std::unique_ptr<float[]> dracoVertex(new float[3]);
    for (int i = 0; (i < attr->indices_map_size()); i++)
    {
        // grab vertex data from draco
        attr->GetValue((draco::AttributeValueIndex)i, dracoVertex.get());

        // dump data into open3d
        Eigen::Vector3d newOpen3dVertex;
        newOpen3dVertex.x() = dracoVertex.get()[0];
        newOpen3dVertex.y() = dracoVertex.get()[1];
        newOpen3dVertex.z() = dracoVertex.get()[2];
        open3dOutMesh->vertices_.push_back(newOpen3dVertex);
    }

    // draco triangles
    for (int i = 0; (i < meshToSave->num_faces()); i++)
    {
        Eigen::Vector3i newOpen3dTriangle;
        // grab draco faces
        newOpen3dTriangle.x() = (meshToSave->face((draco::FaceIndex)i).begin() + 0)->value();
        newOpen3dTriangle.y() = (meshToSave->face((draco::FaceIndex)i).begin() + 1)->value();
        newOpen3dTriangle.z() = (meshToSave->face((draco::FaceIndex)i).begin() + 2)->value();
        open3dOutMesh->triangles_.push_back(newOpen3dTriangle);
    }

    std::cout << "draco\t->\topen3d\ttriangles\tvertices; " << std::endl;
    std::cout << "x\t\t\t"<< meshToSave->num_faces() << "\t\t"<< meshToSave->num_points() << std::endl;
    std::cout << "\t\tx\t"<< open3dOutMesh->triangles_.size() << "\t\t"<< open3dOutMesh->vertices_.size() << std::endl;

    // if (open3d::io::WriteTriangleMesh("/home/allan/draco_encode_cpp/bun_zipper1.ply", *open3dOutMesh));

    open3dOutMesh->ComputeVertexNormals();
    open3d::visualization::DrawGeometries({open3dOutMesh}, "Mesh", 1600, 900);

    printf("success\n");

    return 0;
}