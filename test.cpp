#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <pthread.h>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <numeric>
#include <vector>

// #include "tiny_gltf.h"
#include "/home/allan/Open3D-0.16.0/build/tinygltf/src/ext_tinygltf/tiny_gltf.h"

#include "/home/allan/draco/src/draco/io/obj_encoder.h"
#include "/home/allan/draco/src/draco/compression/encode.h"
#include "/home/allan/draco/src/draco/core/cycle_timer.h"
#include "/home/allan/draco/src/draco/io/file_utils.h"
#include "/home/allan/draco/src/draco/io/mesh_io.h"
// #include "/home/allan/draco/src/draco/io/ply_encoder.h"
#include "/home/allan/draco/src/draco/io/point_cloud_io.h"
#include "/home/allan/draco/src/draco/mesh/mesh.h"

#include "open3d/Open3D.h"
#include "open3d/io/FileFormatIO.h"
// #include "open3d/io/TriangleMeshIO.h"
#include "open3d/utility/FileSystem.h"
#include "open3d/utility/Logging.h"

#include "/home/allan/eigen/Eigen/Core"

// #include "/home/allan/Open3D-0.16.0/cpp/open3d/io/TriangleMeshIO.h"
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

// Hash function for Eigen matrix and vector.
// The code is from `hash_combine` function of the Boost library. See
// http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
    std::size_t operator()(T const &matrix) const
    {
        // Note that it is oblivious to the storage order of Eigen matrix (column- or
        // row-major). It will give you the same hash value for two different matrices if they
        // are the transpose of each other in different storage order.
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i)
        {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

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

// Adapts an array of bytes to an array of T. Will advance of byte_stride each
// elements.
template <typename T>
struct ArrayAdapter
{
    // Pointer to the bytes
    const unsigned char *data_ptr;
    // Number of elements in the array
    const size_t elem_count;
    // Stride in bytes between two elements
    const size_t stride;

    // Construct an array adapter.
    // \param ptr Pointer to the start of the data, with offset applied
    // \param count Number of elements in the array
    // \param byte_stride Stride betweens elements in the array
    ArrayAdapter(const unsigned char *ptr, size_t count, size_t byte_stride)
        : data_ptr(ptr), elem_count(count), stride(byte_stride) {}

    // Returns a *copy* of a single element. Can't be used to modify it.
    T operator[](size_t pos) const
    {
        if (pos >= elem_count)
            throw std::out_of_range(
                "Tried to access beyond the last element of an array "
                "adapter with count " +
                std::to_string(elem_count) +
                " while getting element number " + std::to_string(pos));
        return *(reinterpret_cast<const T *>(data_ptr + pos * stride));
    }
};

// Interface of any adapted array that returns integer data
struct IntArrayBase
{
    virtual ~IntArrayBase() = default;
    virtual unsigned int operator[](size_t) const = 0;
    virtual size_t size() const = 0;
};

// An array that loads integer types, and returns them as int
template <class T>
struct IntArray : public IntArrayBase
{
    ArrayAdapter<T> adapter;

    IntArray(const ArrayAdapter<T> &a) : adapter(a) {}
    unsigned int operator[](size_t position) const override
    {
        return static_cast<unsigned int>(adapter[position]);
    }

    size_t size() const override { return adapter.elem_count; }
};

open3d::io::FileGeometry ReadFileGeometryTypeGLTF(const std::string &path)
{
    return open3d::io::FileGeometry(open3d::io::CONTAINS_TRIANGLES | open3d::io::CONTAINS_POINTS);
}

bool ReadTriangleMeshFromGLTF1(const std::string &filename,
                               open3d::geometry::TriangleMesh &mesh,
                               const open3d::io::ReadTriangleMeshOptions &params /*={}*/)
{
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string warn;
    std::string err;

    std::string filename_ext =
        open3d::utility::filesystem::GetFileExtensionInLowerCase(filename);
    bool ret;
    if (filename_ext == "glb")
    {
        ret = loader.LoadBinaryFromFile(&model, &err, &warn, filename.c_str());
    }
    else
    {
        ret = loader.LoadASCIIFromFile(&model, &err, &warn, filename.c_str());
    }

    if (!warn.empty() || !err.empty())
    {
        open3d::utility::LogWarning("Read GLTF failed: unable to open file {}",
                                    filename);
    }
    if (!ret)
    {
        return false;
    }

    if (model.meshes.size() > 1)
    {
        open3d::utility::LogInfo(
            "The file contains more than one mesh. All meshes will be "
            "loaded as a single mesh.");
    }

    mesh.Clear();
    open3d::geometry::TriangleMesh mesh_temp;
    for (const tinygltf::Node &gltf_node : model.nodes)
    {
        if (gltf_node.mesh != -1)
        {
            mesh_temp.Clear();
            const tinygltf::Mesh &gltf_mesh = model.meshes[gltf_node.mesh];

            for (const tinygltf::Primitive &primitive : gltf_mesh.primitives)
            {
                for (const auto &attribute : primitive.attributes)
                {
                    if (attribute.first == "POSITION")
                    {
                        tinygltf::Accessor &positions_accessor =
                            model.accessors[attribute.second];
                        tinygltf::BufferView &positions_view =
                            model.bufferViews[positions_accessor
                                                  .bufferView];
                        const tinygltf::Buffer &positions_buffer =
                            model.buffers[positions_view.buffer];
                        const float *positions = reinterpret_cast<const float *>(
                            &positions_buffer
                                 .data[positions_view.byteOffset +
                                       positions_accessor.byteOffset]);

                        for (size_t i = 0; i < positions_accessor.count; ++i)
                        {
                            mesh_temp.vertices_.push_back(Eigen::Vector3d(
                                positions[i * 3 + 0], positions[i * 3 + 1],
                                positions[i * 3 + 2]));
                        }
                    }

                    if (attribute.first == "NORMAL")
                    {
                        tinygltf::Accessor &normals_accessor =
                            model.accessors[attribute.second];
                        tinygltf::BufferView &normals_view =
                            model.bufferViews[normals_accessor.bufferView];
                        const tinygltf::Buffer &normals_buffer =
                            model.buffers[normals_view.buffer];
                        const float *normals = reinterpret_cast<const float *>(
                            &normals_buffer
                                 .data[normals_view.byteOffset +
                                       normals_accessor.byteOffset]);

                        for (size_t i = 0; i < normals_accessor.count; ++i)
                        {
                            mesh_temp.vertex_normals_.push_back(Eigen::Vector3d(
                                normals[i * 3 + 0], normals[i * 3 + 1],
                                normals[i * 3 + 2]));
                        }
                    }

                    if (attribute.first == "COLOR_0")
                    {
                        tinygltf::Accessor &colors_accessor =
                            model.accessors[attribute.second];
                        tinygltf::BufferView &colors_view =
                            model.bufferViews[colors_accessor.bufferView];
                        const tinygltf::Buffer &colors_buffer =
                            model.buffers[colors_view.buffer];

                        size_t byte_stride = colors_view.byteStride;
                        if (byte_stride == 0)
                        {
                            // According to glTF 2.0 specs:
                            // When byteStride==0, it means that accessor
                            // elements are tightly packed.
                            byte_stride =
                                colors_accessor.type *
                                tinygltf::GetComponentSizeInBytes(
                                    colors_accessor.componentType);
                        }
                        switch (colors_accessor.componentType)
                        {
                        case TINYGLTF_COMPONENT_TYPE_FLOAT:
                        {
                            for (size_t i = 0; i < colors_accessor.count;
                                 ++i)
                            {
                                const float *colors =
                                    reinterpret_cast<const float *>(
                                        colors_buffer.data.data() +
                                        colors_view.byteOffset +
                                        i * byte_stride);
                                mesh_temp.vertex_colors_.emplace_back(
                                    colors[0], colors[1], colors[2]);
                            }
                            break;
                        }
                        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                        {
                            double max_val = (double)
                                std::numeric_limits<uint8_t>::max();
                            for (size_t i = 0; i < colors_accessor.count;
                                 ++i)
                            {
                                const uint8_t *colors =
                                    reinterpret_cast<const uint8_t *>(
                                        colors_buffer.data.data() +
                                        colors_view.byteOffset +
                                        i * byte_stride);
                                mesh_temp.vertex_colors_.emplace_back(
                                    colors[0] / max_val,
                                    colors[1] / max_val,
                                    colors[2] / max_val);
                            }
                            break;
                        }
                        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                        {
                            double max_val = (double)
                                std::numeric_limits<uint16_t>::max();
                            for (size_t i = 0; i < colors_accessor.count;
                                 ++i)
                            {
                                const uint16_t *colors =
                                    reinterpret_cast<const uint16_t *>(
                                        colors_buffer.data.data() +
                                        colors_view.byteOffset +
                                        i * byte_stride);
                                mesh_temp.vertex_colors_.emplace_back(
                                    colors[0] / max_val,
                                    colors[1] / max_val,
                                    colors[2] / max_val);
                            }
                            break;
                        }
                        default:
                        {
                            open3d::utility::LogWarning(
                                "Unrecognized component type for "
                                "vertex colors");
                            break;
                        }
                        }
                    }
                }

                // Load triangles
                std::unique_ptr<IntArrayBase> indices_array_pointer = nullptr;
                {
                    const tinygltf::Accessor &indices_accessor =
                        model.accessors[primitive.indices];
                    const tinygltf::BufferView &indices_view =
                        model.bufferViews[indices_accessor.bufferView];
                    const tinygltf::Buffer &indices_buffer =
                        model.buffers[indices_view.buffer];
                    const auto data_address = indices_buffer.data.data() +
                                              indices_view.byteOffset +
                                              indices_accessor.byteOffset;
                    const auto byte_stride =
                        indices_accessor.ByteStride(indices_view);
                    const auto count = indices_accessor.count;

                    // Allocate the index array in the pointer-to-base
                    // declared in the parent scope
                    switch (indices_accessor.componentType)
                    {
                    case TINYGLTF_COMPONENT_TYPE_BYTE:
                        indices_array_pointer =
                            std::unique_ptr<IntArray<char>>(
                                new IntArray<char>(
                                    ArrayAdapter<char>(
                                        data_address, count,
                                        byte_stride)));
                        break;
                    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                        indices_array_pointer =
                            std::unique_ptr<IntArray<unsigned char>>(
                                new IntArray<unsigned char>(
                                    ArrayAdapter<unsigned char>(
                                        data_address, count,
                                        byte_stride)));
                        break;
                    case TINYGLTF_COMPONENT_TYPE_SHORT:
                        indices_array_pointer =
                            std::unique_ptr<IntArray<short>>(
                                new IntArray<short>(
                                    ArrayAdapter<short>(
                                        data_address, count,
                                        byte_stride)));
                        break;
                    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                        indices_array_pointer = std::unique_ptr<
                            IntArray<unsigned short>>(
                            new IntArray<unsigned short>(
                                ArrayAdapter<unsigned short>(
                                    data_address, count,
                                    byte_stride)));
                        break;
                    case TINYGLTF_COMPONENT_TYPE_INT:
                        indices_array_pointer =
                            std::unique_ptr<IntArray<int>>(
                                new IntArray<int>(ArrayAdapter<int>(
                                    data_address, count,
                                    byte_stride)));
                        break;
                    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
                        indices_array_pointer =
                            std::unique_ptr<IntArray<unsigned int>>(
                                new IntArray<unsigned int>(
                                    ArrayAdapter<unsigned int>(
                                        data_address, count,
                                        byte_stride)));
                        break;
                    default:
                        break;
                    }
                    const auto &indices = *indices_array_pointer;

                    switch (primitive.mode)
                    {
                    case TINYGLTF_MODE_TRIANGLES:
                        for (size_t i = 0; i < indices_accessor.count;
                             i += 3)
                        {
                            mesh_temp.triangles_.push_back(Eigen::Vector3i(
                                indices[i], indices[i + 1],
                                indices[i + 2]));
                        }
                        break;
                    case TINYGLTF_MODE_TRIANGLE_STRIP:
                        for (size_t i = 2; i < indices_accessor.count;
                             ++i)
                        {
                            mesh_temp.triangles_.push_back(Eigen::Vector3i(
                                indices[i - 2], indices[i - 1],
                                indices[i]));
                        }
                        break;
                    case TINYGLTF_MODE_TRIANGLE_FAN:
                        for (size_t i = 2; i < indices_accessor.count;
                             ++i)
                        {
                            mesh_temp.triangles_.push_back(Eigen::Vector3i(
                                indices[0], indices[i - 1],
                                indices[i]));
                        }
                        break;
                    }
                }
            }

            if (gltf_node.matrix.size() > 0)
            {
                std::vector<double> matrix = gltf_node.matrix;
                Eigen::Matrix4d transform =
                    Eigen::Map<Eigen::Matrix4d>(&matrix[0], 4, 4);
                mesh_temp.Transform(transform);
            }
            else
            {
                // The specification states that first the scale is
                // applied to the vertices, then the rotation, and then the
                // translation.
                if (gltf_node.scale.size() > 0)
                {
                    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
                    transform(0, 0) = gltf_node.scale[0];
                    transform(1, 1) = gltf_node.scale[1];
                    transform(2, 2) = gltf_node.scale[2];
                    mesh_temp.Transform(transform);
                }
                if (gltf_node.rotation.size() > 0)
                {
                    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
                    // glTF represents a quaternion as qx, qy, qz, qw, while
                    // Eigen::Quaterniond orders the parameters as qw, qx,
                    // qy, qz.
                    transform.topLeftCorner<3, 3>() =
                        Eigen::Quaterniond(gltf_node.rotation[3],
                                           gltf_node.rotation[0],
                                           gltf_node.rotation[1],
                                           gltf_node.rotation[2])
                            .toRotationMatrix();
                    mesh_temp.Transform(transform);
                }
                if (gltf_node.translation.size() > 0)
                {
                    mesh_temp.Translate(Eigen::Vector3d(
                        gltf_node.translation[0], gltf_node.translation[1],
                        gltf_node.translation[2]));
                }
            }
            mesh += mesh_temp;
        }
    }

    return true;
}

int main(int argc, char const *argv[])
{

    draco::EncoderBuffer meshBuffer;

    std::unique_ptr<draco::PointCloud> pc;
    draco::Mesh *mesh = nullptr;

    // auto maybe_mesh = draco::ReadMeshFromFile("/home/allan/draco_encode_cpp/bun_zipper.ply", false);
    auto maybe_mesh = draco::ReadMeshFromFile("/home/allan/draco_encode_cpp/example2.obj", false);
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
    sprintf(outPath, "/home/allan/draco_encode_cpp/example2_draco.obj");

    // file read into draco
    // PlyEncoderCustom plyEncoderCustom;
    // draco::PlyEncoder plyEncoder;

    // draco::ObjEncoder objEncoder;
    // objEncoder.EncodeToBuffer()

    // FILE *pFile;
    // char empty[meshBuffer.size() * sizeof(char) * 10] = {0};
    // pFile = fmemopen(NULL, totalRead * sizeof(char) * 10, "w+");
    // int size = meshBuffer.size() * sizeof(char) * 2;
    // pFile = fmemopen(NULL,size , "w+");
    // bool success = plyEncoderCustom.EncodeToFileBuffer(*meshToSave.get(), pFile);
    // bool success = plyEncoder.EncodeToFile(*meshToSave, outPath);
    // bool success = objEncoder.EncodeToFile(*meshToSave, outPath);
    // FILE* pFile2 = fopen("/home/allan/draco_encode_cpp/bun_zipper5.ply", "w+");

    printf("mesh num attributes: %d\n", meshToSave->num_attributes());

    // cout << "mesh attribute: " << meshToSave->GetAttributeElementType(0) << std::endl;
    cout << "mesh number of faces: " << meshToSave->num_faces() << std::endl;
    cout << "mesh number of points: " << meshToSave->num_points() << std::endl;
    // cout << "mesh number of points: " << meshToSave->face() << std::endl;

    // draco triangles
    // std::cout << "draco triangles: " << std::endl;
    // for (int i = 0; (i < meshToSave->num_faces()) && (i < 10); i++)
    // {
    //     std::cout << (meshToSave->face((draco::FaceIndex)i).begin())->value() << std::endl;
    //     std::cout << (meshToSave->face((draco::FaceIndex)i).begin() + 1)->value() << std::endl;
    //     std::cout << (meshToSave->face((draco::FaceIndex)i).begin() + 2)->value() << std::endl;
    //     std::cout << std::endl;
    // }

    // std::cout << "attr map size: " << attr->indices_map_size() << std::endl;
    // int outData[10]={0};
    // std::array<T, att_components_t>
    // std::array<int, 3> outData;
    // draco vertices
    // std::cout << "draco vertices: " << std::endl;
    // const std::unique_ptr<float[]> att_val(new float[3]);
    // for (int i = 0; (i < attr->indices_map_size()) && (i < 10); i++)
    // {
    //     attr->GetValue((draco::AttributeValueIndex)i, att_val.get());
    //     std::cout << att_val.get()[0] << " " << att_val.get()[1] << " " << att_val.get()[2] << std::endl;
    //     std::cout << std::endl;
    // }

    // char outBuffer[size] = {0};
    // fread(outBuffer, size, 1 , pFile);
    // for(int i = 0; i < size; i++){
    //     std::cout << outBuffer[i] << std::endl;
    // }
    // fwrite(outBuffer, size, 1, pFile2);
    // fclose(pFile2);

    // bool success = plyEncoder.EncodeToFile(*mesh, outPath);

    // auto mesh_ptr = std::make_shared<open3d::geometry::TriangleMesh>();

    // if (open3d::io::ReadTriangleMesh("/home/allan/draco_encode_cpp/example2_draco.obj", *mesh_ptr))
    // {
    //     // open3d::utility::LogInfo("Successfully read {}\n", "/home/allan/draco_encode_cpp/bun_zipper.ply");
    // }
    // else
    // {
    //     open3d::utility::LogWarning("Failed to read {}\n\n", "/home/allan/draco_encode_cpp/bun_zipper.ply");
    //     // return 1;
    // }
    // (open3d::io::WriteTriangleMesh("/home/allan/draco_encode_cpp/bun_zipperOpen3d.ply", *mesh_ptr));

    // https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
    // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    // std::cout << "mesh triangles dimension: " << mesh_ptr->triangles_.begin().base()->NumDimensions << std::endl;

    // std::cout << "open3d vertices: " << std::endl
    //           << mesh_ptr->vertices_.begin().base()->format(CleanFmt) << std::endl;
    // std::cout << mesh_ptr->vertices_.begin().base()->x() << std::endl;

    // std::cout << "open3d triangles: " << std::endl;

    // std::ofstream open3dFile;
    // open3dFile.open("/home/allan/draco_encode_cpp/open3dFile.csv");
    // std::vector<Eigen::Vector3d>::iterator open3dVertices = mesh_ptr->vertices_.begin();
    // while (open3dVertices != mesh_ptr->vertices_.end())
    // {
    // open3dFile << open3dVertices.base()->x() << ",";
    // open3dFile << open3dVertices.base()->y() << ",";
    // open3dFile << open3dVertices.base()->z() << "\n";

    // std::cout << std::endl;
    // std::cout << open3dVertices.base()->format(CleanFmt) << std::endl;
    //     open3dVertices++;
    // }

    // std::vector<Eigen::Vector3i>::iterator open3dTriangles = mesh_ptr->triangles_.begin();

    // while (open3dTriangles != mesh_ptr->triangles_.end())
    // {
    //     // std::cout << std::endl;
    //     // std::cout << open3dTriangles.base()->format(CleanFmt) << std::endl;
    //     open3dFile << open3dTriangles.base()->x() << ",";
    //     open3dFile << open3dTriangles.base()->y() << ",";
    //     open3dFile << open3dTriangles.base()->z() << "\n";
    //     open3dTriangles++;
    // }

    // open3dFile.close();

    // draco to open3d
    // auto open3dOutMesh = std::make_shared<open3d::geometry::TriangleMesh>();
    // open3d::io::ReadTriangleMesh("/home/allan/draco_encode_cpp/example2_draco.obj", *mesh_ptr);

    // open3d::geometry::TriangleMesh open3dOutMesh;
    // write vertices first

    // std::ofstream dracoFile;
    // dracoFile.open("/home/allan/draco_encode_cpp/dracoFile.csv");

    // vector to hold all vertices info

    std::vector<Eigen::Vector3d> allVertices;
    std::vector<Eigen::Vector3d> allVerticesToKeep;
    const draco::PointAttribute *attr;
    for (int i = 0; i < meshToSave->num_attributes(); i++)
    {
        attr = meshToSave->GetAttributeByUniqueId(i);
        cout << attr->TypeToString(attr->attribute_type()) << endl;

        // draco loop
        const std::unique_ptr<double[]> dracoVertex(new double[3]);
        for (int j = 0; (j < attr->indices_map_size()); j++)
        {
            // grab vertex data from draco
            // need to convert value to double before extracting
            attr->ConvertValue((draco::AttributeValueIndex)j, attr->num_components(), dracoVertex.get());
            attr->GetValue((draco::AttributeValueIndex)j, dracoVertex.get());

            // dump data into open3d
            // open3d can accept vertex, vertex normal, and vertex color
            Eigen::Vector3d newOpen3dVertex;
            newOpen3dVertex.x() = (double)dracoVertex.get()[0];
            newOpen3dVertex.y() = (double)dracoVertex.get()[1];
            newOpen3dVertex.z() = (double)dracoVertex.get()[2];
            // std::cout << newOpen3dVertex.y() << std::endl;
            // std::cout << dracoVertex.get()[2]<< std::endl;

            if ((strcmp(attr->TypeToString(attr->attribute_type()).c_str(), "POSITION") == 0) ||
                (strcmp(attr->TypeToString(attr->attribute_type()).c_str(), "GENERIC") == 0))
            {
                // open3dOutMesh->vertices_.push_back(*newOpen3dVertex);
                allVerticesToKeep.push_back(newOpen3dVertex);

                // std::cout << newOpen3dVertex.z() << std::endl;
                // std::cout << "added to open3d vertices!" << std::endl;
            }

            // cout << attr->TypeToString(attr->attribute_type()) << endl;
            // open3dOutMesh->vertex_normals_.push_back(*newOpen3dVertex);
            allVertices.push_back(newOpen3dVertex);
            // std::cout << "added to open3d vertex normals!" << std::endl;

            // std::cout << dracoVertex.get()[0] << "," << dracoVertex.get()[1] << "," << dracoVertex.get()[2] << "\n";
        }
    }

    // create a new vector of vectors to store real point data
    std::vector<std::vector<Eigen::Vector3d>> allFaces;

    // draco triangles
    for (int i = 0; (i < meshToSave->num_faces()) && (i < 10); i++)
    {

        // grab draco faces
        // Eigen::Vector3i newOpen3iTriangle;
        // newOpen3iTriangle.x() = (int)(meshToSave->face((draco::FaceIndex)i).begin() + 0)->value();
        // newOpen3iTriangle.y() = (int)(meshToSave->face((draco::FaceIndex)i).begin() + 1)->value();
        // newOpen3iTriangle.z() = (int)(meshToSave->face((draco::FaceIndex)i).begin() + 2)->value();

        std::vector<Eigen::Vector3d> newFace;

        // find the coordinate values by indexing all the vertices
        // attr->mapped_index();
        // tmpX = allVertices[(int)(meshToSave->face((draco::FaceIndex)i).begin() + 0)->value()];
        // tmpY = allVertices[(int)(meshToSave->face((draco::FaceIndex)i).begin() + 1)->value()];
        // tmpZ = allVertices[(int)(meshToSave->face((draco::FaceIndex)i).begin() + 2)->value()];
        // if(isdigit((meshToSave->face((draco::FaceIndex)i)[0]))){
        // }
        // int ptA = (int)(meshToSave->face((draco::FaceIndex)i)[0]);
        // int ptB = (int)(meshToSave->face((draco::FaceIndex)i)[1]);
        // int ptC = (int)(meshToSave->face((draco::FaceIndex)i)[2]);
        attr = meshToSave->GetAttributeByUniqueId(0);
        float out1;
        float out2;
        float out3;
        attr->GetMappedValue((draco::PointIndex)meshToSave->face((draco::FaceIndex)i)[0].value(), &out1);
        attr->GetMappedValue((draco::PointIndex)meshToSave->face((draco::FaceIndex)i)[1].value(), &out2);
        attr->GetMappedValue((draco::PointIndex)meshToSave->face((draco::FaceIndex)i)[2].value(), &out3);
        
        std::cout << meshToSave->face((draco::FaceIndex)i)[0].value() << " : " << out1 << " ";
        std::cout << meshToSave->face((draco::FaceIndex)i)[1].value() << " : " << out2 << " ";
        std::cout << meshToSave->face((draco::FaceIndex)i)[2].value() << " : " << out3 << " ";
        

        std::cout << std::endl;
        // std::cout << meshToSave->face((draco::FaceIndex)i)[0].value() << " ";
        // std::cout << meshToSave->face((draco::FaceIndex)i)[1].value() << " ";
        // std::cout << meshToSave->face((draco::FaceIndex)i)[2].value() << " ";
        // std::cout << std::endl;

        Eigen::Vector3d tmpX;
        Eigen::Vector3d tmpY;
        Eigen::Vector3d tmpZ;
        // std::cout << (int)(meshToSave->face((draco::FaceIndex)i).begin() + 0)->value() << " ";
        // std::cout << (int)(meshToSave->face((draco::FaceIndex)i).begin() + 1)->value() << " ";
        // std::cout << (int)(meshToSave->face((draco::FaceIndex)i).begin() + 2)->value() << " ";
        // std::cout << std::endl;

        // now put these temporary indecies into a vector and store in all faces
        newFace.push_back(tmpX);
        newFace.push_back(tmpY);
        newFace.push_back(tmpZ);

        // now place the new face in all faces
        allFaces.push_back(newFace);

        // newOpen3dTriangle = open3dOutMesh->GetOrderedTriangle(
        //     (meshToSave->face((draco::FaceIndex)i).begin() + 0)->value(),
        //     (meshToSave->face((draco::FaceIndex)i).begin() + 1)->value(),
        //     (meshToSave->face((draco::FaceIndex)i).begin() + 2)->value());
        // open3dOutMesh->triangles_.push_back(*newOpen3iTriangle);

        // dracoFile << (meshToSave->face((draco::FaceIndex)i).begin() + 0)->value() << "," ;
        // dracoFile << (meshToSave->face((draco::FaceIndex)i).begin() + 1)->value() << "," ;
        // dracoFile << (meshToSave->face((draco::FaceIndex)i).begin() + 2)->value() << "\n";
    }
    std::cout << "all face size: " << allFaces.size() << std::endl;
    // dracoFile.close();

    // now create the open3d mesh to enter data into
    auto open3dOutMesh = std::make_shared<open3d::geometry::TriangleMesh>();

    // now put all the vertices in the right place
    // vect2.assign(vect1.begin(), vect1.end());
    // manual copying of data
    for (auto itr = allVerticesToKeep.begin(); itr != allVerticesToKeep.end(); itr++)
    {
        open3dOutMesh->vertices_.push_back(*itr);
        // std::cout << itr->x() << " " << itr->y() << " " << itr->z() << std::endl;
    }
    std::cout << "open3doutmesh vertices size: " << open3dOutMesh->vertices_.size() << std::endl;

    // open3dOutMesh->vertices_.assign(allVertices.begin(), allVertices.end());
    // open3dOutMesh->vertices_ = allVertices;

    // construct hashmap to search for indices
    unordered_map<Eigen::Vector3d, int, matrix_hash<Eigen::Vector3d>> faceMap;

    int count = 1;
    for (auto itr = open3dOutMesh->vertices_.begin(); itr != open3dOutMesh->vertices_.end(); itr++)
    {
        faceMap[*itr] = count;
        count++;
    }

    // now go and reconstruct the correct triangle meshes
    // loop through all triangles
    for (auto itr = allFaces.begin(); itr != allFaces.end(); itr++)
    {

        // get point A and find its index in the stored vertices
        auto vertexA = faceMap.find((*itr)[0]);
        if (vertexA == faceMap.end())
        {
            std::cout << "could not find vector " << std::endl;
            continue;
        }

        // now repeat for B and C
        auto vertexB = faceMap.find((*itr)[1]);
        if (vertexB == faceMap.end())
        {
            std::cout << "could not find vector " << std::endl;
            continue;
        }

        auto vertexC = faceMap.find((*itr)[2]);
        if (vertexC == faceMap.end())
        {
            std::cout << "could not find vector " << std::endl;
            continue;
        }

        // std::cout << "thinking " << std::endl;
        Eigen::Vector3i newOpen3iTriangle(vertexA->second, vertexB->second, vertexC->second);

        // now store the face with the correct vertex indecies in the mesh
        open3dOutMesh->triangles_.push_back(newOpen3iTriangle);
    }

    // auto open3dOutMesh2 = std::make_shared<open3d::geometry::TriangleMesh>();
    // open3d::io::ReadTriangleMeshOptions opt;
    // if (open3d::io::WriteTriangleMesh("/home/allan/draco_encode_cpp/bun_zipper1.ply", *open3dOutMesh));
    // open3d::io::ReadTriangleMeshFromGLTF("/home/allan/draco_encode_cpp/Buggy.glb",*open3dOutMesh2, opt);
    // open3d::io::ReadTriangleMeshFromOBJ("/home/allan/draco_encode_cpp/example2.obj",*open3dOutMesh2, opt);

    // open3dOutMesh2->ComputeVertexNormals();
    open3dOutMesh->ComputeVertexNormals();
    // open3dOutMesh->ComputeTriangleNormals();
    // mesh_ptr->ComputeVertexNormals();

    std::cout << "draco\t->\topen3d\ttriangles\tvertices; " << std::endl;
    std::cout << "x\t\t\t" << meshToSave->num_faces() << "\t\t" << meshToSave->num_points() << std::endl;
    std::cout << "\t\tx\t" << open3dOutMesh->triangles_.size() << "\t\t" << open3dOutMesh->vertices_.size() << std::endl;

    // open3d::io::WriteTriangleMeshToGLTF("/home/allan/draco_encode_cpp/Buggy1.glb",*open3dOutMesh2, true, true, false, true, false, false);
    // open3d::io::WriteTriangleMeshToGLTF("/home/allan/draco_encode_cpp/Buggy1.glb",*open3dOutMesh2, false, true, false, true, false, false);
    // open3d::io::WriteTriangleMeshToGLTF("/home/allan/draco_encode_cpp/Buggy2.glb",*open3dOutMesh2, false, true, true, true, true, false);
    open3d::io::WriteTriangleMeshToOBJ("/home/allan/draco_encode_cpp/example2_.obj", *open3dOutMesh, false, false, true, false, false, false);

    // open3d::visualization::DrawGeometries({open3dOutMesh2}, "Mesh", 1600, 900);
    open3d::visualization::DrawGeometries({open3dOutMesh}, "Mesh", 1600, 900);
    // open3d::visualization::DrawGeometries({mesh_ptr}, "Mesh", 1600, 900);
    // open3d::visualization::Draw()

    printf("success\n");

    return 0;
}