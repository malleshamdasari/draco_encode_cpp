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

// #include "/home/allan/Open3D-0.16.0/build/tinygltf/src/ext_tinygltf/tiny_gltf.h"

#include "../draco/src/draco/io/obj_encoder.h"
#include "../draco/src/draco/compression/encode.h"
#include "../draco/src/draco/core/cycle_timer.h"
#include "../draco/src/draco/io/file_utils.h"
#include "../draco/src/draco/io/mesh_io.h"
#include "../draco/src/draco/io/point_cloud_io.h"
#include "../draco/src/draco/mesh/mesh.h"
#include "../draco/src/draco/mesh/corner_table.h"
#include "../draco/src/draco/attributes/point_attribute.h"
#include "../draco/src/draco/mesh/triangle_soup_mesh_builder.h"
#include "../draco/src/draco/io/ply_encoder.h"

#include "open3d/Open3D.h"
#include "open3d/io/FileFormatIO.h"
#include "open3d/utility/FileSystem.h"
#include "open3d/utility/Logging.h"
#include "open3d/geometry/TriangleMesh.h"

#include "Eigen/Core"



using namespace std;

using std::chrono::duration;
using std::chrono::high_resolution_clock;
using std::chrono::system_clock;
using std::chrono::time_point;
high_resolution_clock::time_point start_time = high_resolution_clock::now();
int time_first = 1;
bool enableDebugging = 0;

duration<double, std::milli> delta(std::string msg = "")
{
    // duration<double, std::milli> t2 = (end2 - start2) / 1000;
    duration<double, std::milli> del;
    // time_point<system_clock,duration<double>> zero_{};
    int silent = 0;
    if (msg == "")
    {
        silent = 1;
    }
    if (time_first)
    {
        start_time = high_resolution_clock::now();
        time_first = 0;
        del = (high_resolution_clock::now() - high_resolution_clock::now()) / 1000;
    }
    else
    {
        del = (high_resolution_clock::now() - start_time) / 1000;
        if (!silent)
        {
            std::cout << "  " << msg << ": " << del.count() << " s" << std::endl;
        }
        start_time = high_resolution_clock::now();
    }
    return del;
}

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
        for (size_t i = 0; i < (unsigned long)matrix.size(); ++i)
        {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

bool AreSame(double a, double b)
{
    return fabs(a - b) < 0.1;
}

int main(int argc, char const *argv[])
{
    delta();
    auto inOpen3d = std::make_shared<open3d::geometry::TriangleMesh>();
    open3d::io::ReadTriangleMeshOptions inOpt;
    // open3d::io::ReadTriangleMeshFromOBJ("/home/allan/draco_encode_cpp/custom0.obj", *inOpen3d, inOpt);
    open3d::io::ReadTriangleMeshFromOBJ("/home/allan/draco_encode_cpp/example2.obj", *inOpen3d, inOpt);

    // inOpen3d->RemoveDuplicatedVertices();
    
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

    draco::EncoderBuffer meshBuffer;
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
    encoder.EncodeMeshToBuffer(*mesh, &meshBuffer);
    // }
    // else
    // {
    //     encoder.EncodePointCloudToBuffer(*pc, &meshBuffer);
    // }
    // std::cout << "finished encoding " << std::endl;
    draco::DecoderBuffer decoderBuffer;
    decoderBuffer.Init(meshBuffer.data(), meshBuffer.size());

    draco::Decoder decoder;
    std::shared_ptr<draco::Mesh> meshToSave = decoder.DecodeMeshFromBuffer(&decoderBuffer).value();

    // draco::Mesh *meshToSave;
    // meshToSave = mesh;
    // std::shared_ptr<draco::Mesh> meshToSave;
    // meshToSave = mesh;

    // char outPath[1024] = {0};
    // sprintf(outPath, "/home/allan/draco_encode_cpp/custom0_draco.obj");

    // draco::ObjEncoder objEncoder;
    // objEncoder.EncodeToBuffer()
    // bool success = objEncoder.EncodeToFile(*meshToSave, outPath);

    if (enableDebugging)
    {
        printf("mesh num attributes: %d\n", meshToSave->num_attributes());

        // cout << "mesh attribute: " << meshToSave->GetAttributeElementType(0) << std::endl;
        cout << "mesh number of faces: " << meshToSave->num_faces() << std::endl;
        cout << "mesh number of points: " << meshToSave->num_points() << std::endl;

        // https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
        // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        // std::cout << "mesh triangles dimension: " << mesh_ptr->triangles_.begin().base()->NumDimensions << std::endl;

        // vector to hold all vertices info
        std::cout << "draco format: " << std::endl;
    }
    std::vector<Eigen::Vector3d> allVerticesP;
    std::vector<Eigen::Vector3d> allVerticesN;
    const draco::PointAttribute *attr;

    unsigned long count0 = 0;

    for (int i = 0; i < meshToSave->num_attributes(); i++)
    {
        attr = meshToSave->GetAttributeByUniqueId(i);
        if (enableDebugging)
        {
            std::cout << attr->TypeToString(attr->attribute_type()) << std::endl;
        }
        if (enableDebugging)
        {
            std::cout << "size: " << attr->size() << std::endl;
            std::cout << "size indices map size: " << attr->indices_map_size() << std::endl;
        }

        if (enableDebugging)
        {
            count0 = 0;
        }

        for (unsigned long j = 0; (j < attr->size()); j++)
        {
            // grab vertex data from draco
            // need to convert value to double before extracting
            // draco loop
            double dracoVertex[3];
            attr->ConvertValue((draco::AttributeValueIndex((unsigned int)j)), 3, &dracoVertex[0]);
            // dump data into open3d
            // open3d can accept vertex, vertex normal, and vertex color
            Eigen::Vector3d newOpen3dVertex(dracoVertex[0], dracoVertex[1], dracoVertex[2]);

            // only considering position - conains all points (not sure what generic is - same number as position)
            if ((strcmp(attr->TypeToString(attr->attribute_type()).c_str(), "POSITION") == 0))
            {
                allVerticesP.push_back(newOpen3dVertex);
                if (enableDebugging)
                {
                    std::cout << "(" << count0 << ") " << newOpen3dVertex.x() << " " << newOpen3dVertex.y() << " " << newOpen3dVertex.z() << std::endl;
                }
            }
            else if ((strcmp(attr->TypeToString(attr->attribute_type()).c_str(), "NORMAL") == 0))
            {
                allVerticesN.push_back(newOpen3dVertex);
                if (enableDebugging)
                {
                    std::cout << "(" << count0 << ")  " << newOpen3dVertex.x() << " " << newOpen3dVertex.y() << " " << newOpen3dVertex.z() << std::endl;
                }
            }
            else
            {
                if (enableDebugging)
                {
                    std::cout << "attribute not POSITION or NORMAL" << std::endl;
                }
            }
            if (enableDebugging)
            {
                count0++;
            }
        }
    }

    // create a new vector of vectors to store real point data
    // fourth Eigen element (vertex) is the face normal
    std::vector<std::vector<Eigen::Vector3d>> facesWithVerticesAndNormal;
    if (enableDebugging)
    {
        std::cout << "FACES: " << meshToSave->num_faces() << std::endl;
    }
    // loop draco faces and convert every index to the actual vertex value

    // usually, this is not always true
    // attribute 0 = position - MESH_CORNER_ATTRIBUTE
    // attribute 1 = tex_coord - MESH_CORNER_ATTRIBUTE - ignore this
    // attribute 2 = normal - MESH_CORNER_ATTRIBUTE - don't think I need this
    // attribute 3 = generic - MESH_CORNER_ATTRIBUTE - has garbage data
    const draco::PointAttribute *attrP = meshToSave->GetAttributeByUniqueId(0);
    // const draco::PointAttribute *attrN = meshToSave->GetAttributeByUniqueId(1);
    for (unsigned long i = 0; (i < meshToSave->num_faces()); i++)
    {
        // grab draco faces
        // find the coordinate values by indexing all the vertices
        draco::Mesh::Face face = meshToSave->face((draco::FaceIndex((uint32_t)i)));

        double vP0[3];
        double vP1[3];
        double vP2[3];

        attrP->ConvertValue(attrP->mapped_index(face[0]), 3, &vP0[0]);
        attrP->ConvertValue(attrP->mapped_index(face[1]), 3, &vP1[0]);
        attrP->ConvertValue(attrP->mapped_index(face[2]), 3, &vP2[0]);
        if (enableDebugging)
        {
            std::cout << "(" << attrP->mapped_index(face[0]).value() << ")"
                      << " ";
            std::cout << vP0[0] << " " << vP0[1] << " " << vP0[2] << " ";

            std::cout << "(" << attrP->mapped_index(face[1]).value() << ")"
                      << " ";
            std::cout << vP1[0] << " " << vP1[1] << " " << vP1[2] << " ";

            std::cout << "(" << attrP->mapped_index(face[2]).value() << ")"
                      << " ";
            std::cout << vP2[0] << " " << vP2[1] << " " << vP2[2] << " ";
        }

        Eigen::Vector3d tmpX(vP0[0], vP0[1], vP0[2]);
        Eigen::Vector3d tmpY(vP1[0], vP1[1], vP1[2]);
        Eigen::Vector3d tmpZ(vP2[0], vP2[1], vP2[2]);

        std::vector<Eigen::Vector3d> facePN;

        // now put these temporary indecies into a vector and store in all faces
        facePN.push_back(tmpX);
        facePN.push_back(tmpY);
        facePN.push_back(tmpZ);

        // now place the new face in all faces
        facesWithVerticesAndNormal.push_back(facePN);
    }

    // now create the open3d mesh to enter data into
    auto open3dOutMesh = std::make_shared<open3d::geometry::TriangleMesh>();

    // now put all the vertices in the right place
    // manual copying of data
    for (auto itr = allVerticesP.begin(); itr != allVerticesP.end(); itr++)
    {
        // don't think I need to do this copying manually TODO
        // Eigen::Vector3d tmpVec((*itr)[0],(*itr)[1],(*itr)[2]);
        Eigen::Vector3d tmpVec((*itr)[0], (*itr)[1], (*itr)[2]);
        open3dOutMesh->vertices_.push_back(tmpVec);
        // std::cout << itr->x() << " " << itr->y() << " " << itr->z() << std::endl;
    }
    for (auto itr = allVerticesN.begin(); itr != allVerticesN.end(); itr++)
    {
        // don't think I need to do this copying manually TODO
        // Eigen::Vector3d tmpVec((*itr)[0],(*itr)[1],(*itr)[2]);
        Eigen::Vector3d tmpVec((*itr)[0], (*itr)[1], (*itr)[2]);
        open3dOutMesh->vertex_normals_.push_back(tmpVec);
        // std::cout << itr->x() << " " << itr->y() << " " << itr->z() << std::endl;
    }
    if (enableDebugging)
    {
        std::cout << "open3doutmesh vertices size: " << open3dOutMesh->vertices_.size() << std::endl;
        std::cout << "open3doutmesh vertices normal size: " << open3dOutMesh->vertex_normals_.size() << std::endl;
    }

    // construct hashmap to search for indices
    unordered_map<Eigen::Vector3d, unsigned long, matrix_hash<Eigen::Vector3d>> faceMapP;
    unordered_map<Eigen::Vector3d, unsigned long, matrix_hash<Eigen::Vector3d>> faceMapN;

    unsigned long count = 0;
    for (auto itr = open3dOutMesh->vertices_.begin(); itr != open3dOutMesh->vertices_.end(); itr++)
    {
        // std::cout << count << " : ";
        // std::cout << (*itr)[0] << ", " << (*itr)[1] << ", " << (*itr)[2] << "\n";
        Eigen::Vector3d tmpVec((*itr)[0], (*itr)[1], (*itr)[2]);
        faceMapP[tmpVec] = count;
        count++;
    }
    count = 0;
    for (auto itr = open3dOutMesh->vertex_normals_.begin(); itr != open3dOutMesh->vertex_normals_.end(); itr++)
    {
        // std::cout << count << " : ";
        // std::cout << (*itr)[0] << ", " << (*itr)[1] << ", " << (*itr)[2] << "\n";
        Eigen::Vector3d tmpVec((*itr)[0], (*itr)[1], (*itr)[2]);
        faceMapN[tmpVec] = count;
        count++;
    }

    // open3dOutMesh->triangles_.clear();
    // open3dOutMesh->triangle_normals_.clear();
    // now go and reconstruct the correct triangle meshes
    // loop through all triangles
    for (auto itr = facesWithVerticesAndNormal.begin(); itr != facesWithVerticesAndNormal.end(); itr++)
    {

        // get point A and find its index in the stored vertices
        auto vertexA = faceMapP.find((*itr)[0]);
        if (vertexA == faceMapP.end())
        {
            std::cout << "could not find vector " << std::endl;
            continue;
        }

        // now repeat for B and C
        auto vertexB = faceMapP.find((*itr)[1]);
        if (vertexB == faceMapP.end())
        {
            std::cout << "could not find vector " << std::endl;
            continue;
        }

        auto vertexC = faceMapP.find((*itr)[2]);
        if (vertexC == faceMapP.end())
        {
            std::cout << "could not find vector " << std::endl;
            continue;
        }

        // check if its valid
        if (((!AreSame((*itr)[0].x(), vertexA->first[0])) || (!AreSame((*itr)[0].y(), vertexA->first[1])) || (!AreSame((*itr)[0].z(), vertexA->first[2]))))
        {
            std::cout << "looking for:\n " << (*itr)[0] << " "
                      << "and got: \n"
                      << vertexA->first << std::endl;
        }
        if (((!AreSame((*itr)[1].x(), vertexB->first[0])) || (!AreSame((*itr)[1].y(), vertexB->first[1])) || (!AreSame((*itr)[1].z(), vertexB->first[2]))))
        {
            std::cout << "looking for:\n " << (*itr)[1] << " "
                      << "and got: \n"
                      << vertexB->first << std::endl;
        }
        if (((!AreSame((*itr)[2].x(), vertexC->first[0])) || (!AreSame((*itr)[2].y(), vertexC->first[1])) || (!AreSame((*itr)[2].z(), vertexC->first[2]))))
        {
            std::cout << "looking for:\n " << (*itr)[2] << " "
                      << "and got: \n"
                      << vertexC->first << std::endl;
        }

        // index values of open3d vertices P
        // changing the ordering of the vertices changing the faces??? TODO
        Eigen::Vector3i triP((int)vertexA->second, (int)vertexB->second, (int)vertexC->second);

        // for each triangle vertex P there exists a triangle vertex N (but store its actual value, not index)
        // now store the face with the correct vertex indecies in the mesh
        open3dOutMesh->triangles_.push_back(triP);
    }

    auto open3dOutMesh2 = std::make_shared<open3d::geometry::TriangleMesh>();
    open3d::io::ReadTriangleMeshOptions opt;
    // {vertex index}/{vertex texture coordinate index}/{vertex normal index}
    if (enableDebugging)
    {
        open3d::io::ReadTriangleMeshFromOBJ("/home/allan/draco_encode_cpp/custom0_open3d.obj", *open3dOutMesh2, opt);
        open3dOutMesh2->ComputeTriangleNormals(false);
        // open3dOutMesh2->RemoveDuplicatedTriangles();
        // open3dOutMesh2->ComputeVertexNormals(false);
        // open3dOutMesh->RemoveDuplicatedVertices();
        // open3dOutMesh2->RemoveDuplicatedVertices();
    }
    
    open3dOutMesh->ComputeVertexNormals(false);
    // open3dOutMesh->RemoveDuplicatedVertices();
    // open3dOutMesh->RemoveDuplicatedTriangles();
    // open3dOutMesh->ComputeTriangleNormals(false);
    // open3dOutMesh->RemoveDegenerateTriangles();
    // open3dOutMesh->RemoveNonManifoldEdges();

    if (enableDebugging)
    {
        std::cout << std::endl
                  << "open3d format: " << std::endl;
        std::cout << "VERTICES: " << std::endl;
        count = 0;
        for (auto itr = open3dOutMesh->vertices_.begin(); itr != open3dOutMesh->vertices_.end(); itr++)
        {
            std::cout << "(" << count << ")";
            std::cout << (*itr)[0] << " " << (*itr)[1] << " " << (*itr)[2] << "\n";
            count++;
        }
        std::cout << "VERTEX NORMALS: " << std::endl;
        count = 0;
        for (auto itr = open3dOutMesh->vertex_normals_.begin(); itr != open3dOutMesh->vertex_normals_.end(); itr++)
        {
            std::cout << "(" << count << ")";
            std::cout << (*itr)[0] << " " << (*itr)[1] << " " << (*itr)[2] << "\n";
            count++;
        }

        std::cout << "FACES: " << open3dOutMesh->triangles_.size() << std::endl;
        // int count2 =0;
        for (auto itr = open3dOutMesh->triangles_.begin(); itr != open3dOutMesh->triangles_.end(); itr++)
        {
            std::cout << "(" << (*itr)[0] << ") " << open3dOutMesh->vertices_[(*itr)[0]][0] << " " << open3dOutMesh->vertices_[(*itr)[0]][1] << " " << open3dOutMesh->vertices_[(*itr)[0]][2] << " ";
            std::cout << "(" << (*itr)[1] << ") " << open3dOutMesh->vertices_[(*itr)[1]][0] << " " << open3dOutMesh->vertices_[(*itr)[1]][1] << " " << open3dOutMesh->vertices_[(*itr)[1]][2] << " ";
            std::cout << "(" << (*itr)[2] << ") " << open3dOutMesh->vertices_[(*itr)[2]][0] << " " << open3dOutMesh->vertices_[(*itr)[2]][1] << " " << open3dOutMesh->vertices_[(*itr)[2]][2] << std::endl;
        }
        std::cout << "FACE NORMALS" << std::endl;
        for (auto itr = open3dOutMesh->triangle_normals_.begin(); itr != open3dOutMesh->triangle_normals_.end(); itr++)
        {
            std::cout << (*itr)[0] << " " << (*itr)[1] << " " << (*itr)[2] << std::endl;
        }

        std::cout << std::endl
                  << "open3d format REFERENCE: " << std::endl;
        std::cout << "VERTICES: " << std::endl;
        count = 0;
        for (auto itr = open3dOutMesh2->vertices_.begin(); itr != open3dOutMesh2->vertices_.end(); itr++)
        {
            std::cout << "(" << count << ")";
            std::cout << (*itr)[0] << " " << (*itr)[1] << " " << (*itr)[2] << "\n";
            count++;
        }
        std::cout << "VERTEX NORMALS: " << std::endl;
        count = 0;
        for (auto itr = open3dOutMesh2->vertex_normals_.begin(); itr != open3dOutMesh2->vertex_normals_.end(); itr++)
        {
            std::cout << "(" << count << ")";
            std::cout << (*itr)[0] << " " << (*itr)[1] << " " << (*itr)[2] << "\n";
            count++;
        }
        std::cout << "FACES: " << open3dOutMesh2->triangles_.size() << std::endl;
        for (auto itr = open3dOutMesh2->triangles_.begin(); itr != open3dOutMesh2->triangles_.end(); itr++)
        {
            std::cout << "(" << (*itr)[0] << ") " << open3dOutMesh2->vertices_[(*itr)[0]][0] << " " << open3dOutMesh2->vertices_[(*itr)[0]][1] << " " << open3dOutMesh2->vertices_[(*itr)[0]][2] << " ";
            std::cout << "(" << (*itr)[1] << ") " << open3dOutMesh2->vertices_[(*itr)[1]][0] << " " << open3dOutMesh2->vertices_[(*itr)[1]][1] << " " << open3dOutMesh2->vertices_[(*itr)[1]][2] << " ";
            std::cout << "(" << (*itr)[2] << ") " << open3dOutMesh2->vertices_[(*itr)[2]][0] << " " << open3dOutMesh2->vertices_[(*itr)[2]][1] << " " << open3dOutMesh2->vertices_[(*itr)[2]][2] << std::endl;
        }

        std::cout << "FACE NORMALS" << std::endl;

        for (auto itr = open3dOutMesh2->triangle_normals_.begin(); itr != open3dOutMesh2->triangle_normals_.end(); itr++)
        {
            std::cout << (*itr)[0] << " " << (*itr)[1] << " " << (*itr)[2] << std::endl;
        }

        std::cout << "draco\t->\topen3d\ttriangles\tvertices; " << std::endl;
        std::cout << "x\t\t\t" << meshToSave->num_faces() << "\t\t" << meshToSave->num_points() << std::endl;
        std::cout << "\t\tx\t" << open3dOutMesh->triangles_.size() << "\t\t" << open3dOutMesh->vertices_.size() << std::endl;

        std::cout << "draco version: has triangles: " << open3dOutMesh->HasTriangles() << " has triangle normals: " << open3dOutMesh->HasTriangleNormals();
        std::cout << " has vertices: " << open3dOutMesh->HasVertices() << " has vertex normals: " << open3dOutMesh->HasVertexNormals();
        std::cout << " has adj list: " << open3dOutMesh->HasAdjacencyList() << " has triangle uvs: " << open3dOutMesh->HasTriangleUvs() << std::endl;

        std::cout << "open3d version: has triangles: " << open3dOutMesh2->HasTriangles() << " has triangle normals: " << open3dOutMesh2->HasTriangleNormals();
        std::cout << " has vertices: " << open3dOutMesh2->HasVertices() << " has vertex normals: " << open3dOutMesh2->HasVertexNormals();
        std::cout << " has adj list: " << open3dOutMesh2->HasAdjacencyList() << " has triangle uvs: " << open3dOutMesh2->HasTriangleUvs() << std::endl;
        // open3d::io::WriteTriangleMeshToOBJ("/home/allan/draco_encode_cpp/custom0_draco_to_open3d.obj", *open3dOutMesh, false, false, true, false, false, false);
        // open3d::io::WriteTriangleMeshToOBJ("/home/allan/draco_encode_cpp/custom0_open3d.obj", *open3dOutMesh2, false, false, true, false, false, false);

        printf("success\n");
    }
    delta("elapsed time: ");
    open3d::visualization::DrawGeometries({open3dOutMesh}, "Mesh", 1600, 900);
    return 0;
}