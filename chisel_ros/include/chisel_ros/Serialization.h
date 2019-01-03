// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef SERIALIZATION_H_
#define SERIALIZATION_H_

#include <open_chisel/Chunk.h>
#include <chisel_msgs/ChunkMessage.h>

namespace chisel_ros
{
    void FillChunkMessage(chisel::ChunkPtr chunk, chisel_msgs::ChunkMessage* message)
    {
        chisel::ChunkHasher hasher;
        assert(message != nullptr);
        message->header.stamp = ros::Time::now();
        message->defined=true;
        chisel::ChunkID id = chunk->GetID();
        message->ID_x = id.x();
        message->ID_y = id.y();
        message->ID_z = id.z();
        message->spatial_hash = hasher(id);

        message->resolution_meters = chunk->GetVoxelResolutionMeters();

        Eigen::Vector3i size = chunk->GetNumVoxels();
        message->num_voxels_x = size.x();
        message->num_voxels_y = size.y();
        message->num_voxels_z = size.z();

        message->distance_data.reserve(chunk->GetTotalNumVoxels());

        if(chunk->HasColors())
        {
            message->color_data.reserve(chunk->GetTotalNumVoxels());
        }

        const std::vector<chisel::DistVoxel>& voxels = chunk->GetVoxels();

        for (const chisel::DistVoxel& voxel : voxels)
        {
            float sdf = voxel.GetSDF();
            float weight = voxel.GetWeight();
            uint64_t distance_data_sdf;
            distance_data_sdf=*(reinterpret_cast<uint32_t*>(&sdf));
            distance_data_sdf<<=32;
            uint64_t distance_data_weight;
            distance_data_weight=*(reinterpret_cast<uint32_t*>(&weight));

            message->distance_data.push_back(distance_data_sdf | distance_data_weight);
        }

        const std::vector<chisel::ColorVoxel>& colors = chunk->GetColorVoxels();

        for (const chisel::ColorVoxel& voxel : colors)
        {
            uint32_t color_data_red,color_data_green,color_data_blue,color_data_weight;
            color_data_red   =voxel.GetRed();
            color_data_green =voxel.GetBlue();
            color_data_blue  =voxel.GetGreen();
            color_data_weight=voxel.GetWeight();
            message->color_data.push_back
            (
                    color_data_weight
                    | color_data_green   << 8
                    | color_data_green  << 2 * 8
                    | color_data_red << 3 * 8
            );
        }

    }
}


#endif // SERIALIZATION_H_ 
