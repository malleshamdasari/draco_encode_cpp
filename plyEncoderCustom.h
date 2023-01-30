// Copyright 2016 The Draco Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#ifndef PLY_ENCODER_CUSTOM_H_
#define PLY_ENCODER_CUSTOM_H_

#include "draco/core/encoder_buffer.h"
#include "draco/mesh/mesh.h"

// Class for encoding draco::Mesh or draco::PointCloud into the PLY file format.
class PlyEncoderCustom
{
public:
    PlyEncoderCustom();

    // Encodes the mesh or a point cloud  and saves it into a file.
    // Returns false when either the encoding failed or when the file couldn't be
    // opened.
    bool EncodeToFile(const draco::Mesh &mesh, const std::string &file_name);
    bool EncodeToFile(const draco::PointCloud &pc, const std::string &file_name);

    // custom by anesathu
    bool EncodeToFileBuffer(const draco::Mesh &mesh, FILE *pFileBuffer);
    bool EncodeToFileBuffer(const draco::PointCloud &pc, FILE *pFileBuffer);

    // Encodes the mesh or the point cloud into a buffer.
    bool EncodeToBuffer(const draco::PointCloud &pc, draco::EncoderBuffer *out_buffer);
    bool EncodeToBuffer(const draco::Mesh &mesh, draco::EncoderBuffer *out_buffer);

protected:
    bool EncodeInternal();
    draco::EncoderBuffer *buffer() const { return out_buffer_; }
    bool ExitAndCleanup(bool return_value);

private:
    const char *GetAttributeDataType(int attribute);

    draco::EncoderBuffer *out_buffer_;

    const draco::PointCloud *in_point_cloud_;
    const draco::Mesh *in_mesh_;
};

#endif  // DRACO_IO_PLY_ENCODER_H_
