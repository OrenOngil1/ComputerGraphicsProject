// Headless round-trip check for the color-pick id encoding: encode a vertex
// id, quantize each channel to 8 bits as the RGB8 framebuffer would, decode.

#include <cmath>
#include <iostream>

#include "check.h"
#include "../src/render/PickEncoding.h"

// What the GPU does between encode and decode: a [0,1] float channel lands in
// an 8-bit framebuffer as round(value * 255).
static unsigned char quantize(float channel)
{
    return (unsigned char)std::lround(channel * 255.0f);
}

void testPickEncoding()
{
    std::cout << "pick encoding:" << std::endl;

    // Ids straddling every byte boundary of the 24-bit range.
    const unsigned int ids[] = { 0, 1, 254, 255, 256, 65535, 65536, 16777214 };
    bool allRoundTrip = true;
    for (unsigned int id : ids) {
        const glm::vec3 color = encodeVertexId(id);
        const unsigned char px[3] = { quantize(color.r), quantize(color.g), quantize(color.b) };
        if (decodeVertexId(px, 16777215) != (int)id)
            allRoundTrip = false;
    }
    check(allRoundTrip, "decode(quantize(encode(id))) == id across the 24-bit range");

    // The pick pass clears to white, which must decode to a miss for any
    // realistically sized terrain.
    const unsigned char white[3] = { 255, 255, 255 };
    check(decodeVertexId(white, 1000000) == -1, "white background decodes to a miss");

    // The vertexCount guard is half-open: count - 1 is the last valid id.
    const glm::vec3 color = encodeVertexId(500);
    const unsigned char px[3] = { quantize(color.r), quantize(color.g), quantize(color.b) };
    check(decodeVertexId(px, 500) == -1, "id == vertexCount decodes to a miss");
    check(decodeVertexId(px, 501) == 500, "id == vertexCount - 1 decodes to itself");
}
