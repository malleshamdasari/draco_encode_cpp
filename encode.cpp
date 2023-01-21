#include <iostream>

#include "/home/allan/draco/src/draco/compression/encode.h"
#include "/home/allan/draco/src/draco/core/cycle_timer.h"
#include "/home/allan/draco/src/draco/io/file_utils.h"
#include "/home/allan/draco/src/draco/io/mesh_io.h"
#include "/home/allan/draco/src/draco/io/point_cloud_io.h"

// #include <sys/ipc.h>
// #include <sys/msg.h>

using namespace std;

#define MAX 4096

struct mesg_buffer {
        long mesg_type;
        char data[MAX];
} message;

// int sendtomessagercv(const char *buffer, int size)
// {
//         key_t key;
//         int msgid;

//         // ftok to generate unique key
//         key = ftok("meshstreamer", 65);

//         // msgget creates a message queue
//         // and returns identifier
//         msgid = msgget(key, 0666 | IPC_CREAT);
//         message.mesg_type = 1;

//         int idx = 0;
//         while (size > MAX) {
//             memcpy(message.data, buffer+idx, MAX);

//             // msgsnd to send message
//             msgsnd(msgid, &message, sizeof(message), 0);
// 	    printf("Message sent to rcv queue\n");
//             size -= MAX;
//             idx += MAX;
//         }

//         return 0;
// }

int EncodeMeshToFile(const draco::Mesh &mesh, const std::string &file,
                     draco::ExpertEncoder *encoder) {
  draco::CycleTimer timer;
  // Encode the geometry.
  draco::EncoderBuffer buffer;
  timer.Start();
  const draco::Status status = encoder->EncodeToBuffer(&buffer);
  if (!status.ok()) {
    printf("Failed to encode the mesh.\n");
    printf("%s\n", status.error_msg());
    return -1;
  }
  timer.Stop();

  // sendtomessagercv(buffer.data(), buffer.size());

  // Save the encoded geometry into a file.
  if (!draco::WriteBufferToFile(buffer.data(), buffer.size(), file)) {
    printf("Failed to create the output file.\n");
    return -1;
  }
  printf("Encoded mesh saved to %s (%" PRId64 " ms to encode).\n", file.c_str(),
         timer.GetInMs());
  printf("\nEncoded size = %zu bytes\n\n", buffer.size());
  return 0;
}

int EncodePointCloudToFile(const draco::PointCloud &pc, const std::string &file,
                           draco::ExpertEncoder *encoder) {
  draco::CycleTimer timer;
  // Encode the geometry.
  draco::EncoderBuffer buffer;
  timer.Start();
  const draco::Status status = encoder->EncodeToBuffer(&buffer);
  if (!status.ok()) {
    printf("Failed to encode the point cloud.\n");
    printf("%s\n", status.error_msg());
    return -1;
  }
  timer.Stop();
  // Save the encoded geometry into a file.
  if (!draco::WriteBufferToFile(buffer.data(), buffer.size(), file)) {
    printf("Failed to write the output file.\n");
    return -1;
  }
  printf("Encoded point cloud saved to %s (%" PRId64 " ms to encode).\n",
         file.c_str(), timer.GetInMs());
  printf("\nEncoded size = %zu bytes\n\n", buffer.size());
  return 0;
}

int main() {
  cout<<"Hello\n";
  std::unique_ptr<draco::PointCloud> pc;
  draco::Mesh *mesh = nullptr;

  auto maybe_mesh = draco::ReadMeshFromFile("/home/allan/draco_encode_cpp/bun_zipper.ply", false);
  if (!maybe_mesh.ok()) {
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
  if (input_is_mesh) {
    expert_encoder.reset(new draco::ExpertEncoder(*mesh));
  } else {
    expert_encoder.reset(new draco::ExpertEncoder(*pc));
  }
  expert_encoder->Reset(encoder.CreateExpertEncoderOptions(*pc));

  //// Check if there is an attribute that stores polygon edges. If so, we disable
  //// the default prediction scheme for the attribute as it actually makes the
  //// compression worse.
  //const int poly_att_id =
  //    pc->GetAttributeIdByMetadataEntry("name", "added_edges");
  //if (poly_att_id != -1) {
  //  expert_encoder->SetAttributePredictionScheme(
  //      poly_att_id, draco::PredictionSchemeMethod::PREDICTION_NONE);
  //}

  int ret = -1;

  if (input_is_mesh) {
    ret = EncodeMeshToFile(*mesh, "compressed.drc", expert_encoder.get());
  } else {
    ret = EncodePointCloudToFile(*pc, "compressed.drc", expert_encoder.get());
  }

//
//
//
//  // Save to file
//  //int res = EncodeMeshToFile(*mesh, "example.drc", &encoder);
//  //if(res == -1) std::cout << "Failed DRACO !\n";
//
//  draco::CycleTimer timer;
//  // Encode the geometry.
//  draco::EncoderBuffer buffer;
//  timer.Start();
//  const draco::Status status = encoder->EncodeMeshToBuffer(&buffer);
//  if (!status.ok()) {
//    printf("Failed to encode the mesh.\n");
//    printf("%s\n", status.error_msg());
//    return -1;
//  }
//  timer.Stop();
//  printf("Encoded mesh (%" PRId64 " ms to encode).\n", timer.GetInMs());
//  printf("\nEncoded size = %zu bytes\n\n", buffer.size());

  return 0;
}
