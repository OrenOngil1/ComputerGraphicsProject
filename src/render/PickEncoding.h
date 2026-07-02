#pragma once

#include <cstddef>

#include <glm/glm.hpp>

// Both directions of the color-pick pass's id<->color contract. The terrain
// upload bakes encodeVertexId of every vertex index into a per-vertex
// attribute, the pick shader passes it through untouched, and pickVertex
// decodes the pixel under the cursor back into an id.
//
// Packing: the id's low, middle and high bytes land in R, G and B (8 bits per
// channel = up to ~16.7M ids). Each channel is a whole number of 1/255 steps,
// which an RGB8 framebuffer stores exactly, so the round trip is lossless.

// A vertex id as the [0,1] RGB color the pick pass draws it in.
inline glm::vec3 encodeVertexId(unsigned int id)
{
    return glm::vec3((float)( id        & 0xFF),
                     (float)((id >>  8) & 0xFF),
                     (float)((id >> 16) & 0xFF)) / 255.0f;
}

// A read-back pixel's vertex id, or -1 when the pixel encodes none (the white
// pick-pass background, or a pixel that isn't from a pick pass at all).
inline int decodeVertexId(const unsigned char *px, size_t vertexCount)
{
    unsigned int id = px[0] | (px[1] << 8) | (px[2] << 16);
    return id < vertexCount ? (int)id : -1;
}
