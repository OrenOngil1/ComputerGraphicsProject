#shader vertex
#version 330 core

// Color-pick pass: render each triangle in a flat color that encodes the id of
// its provoking vertex, so reading one pixel back tells us which vertex was clicked.
layout(location = 0) in vec3 a_Position;   // attribute 1 (color) is ignored here

// `flat` = no interpolation: every fragment of a triangle gets the provoking
// vertex's value verbatim, so it decodes to one whole vertex id (a smooth/
// interpolated varying would blend three ids into a meaningless color).
flat out vec3 v_Id;

uniform mat4 u_MVP;

// Pack a vertex index into an RGB color (8 bits per channel = up to ~16.7M ids).
vec3 encodeId(int id)
{
    int r =  id        & 0xFF;
    int g = (id >> 8)  & 0xFF;
    int b = (id >> 16) & 0xFF;
    return vec3(r, g, b) / 255.0;
}

void main()
{
    gl_Position = u_MVP * vec4(a_Position, 1.0);
    // gl_VertexID under indexed drawing is the value pulled from the element
    // buffer -- i.e. the mesh vertex index -- so no CPU-side color table is needed.
    v_Id = encodeId(gl_VertexID);
}

#shader fragment
#version 330 core

layout(location = 0) out vec4 FragColor;

flat in vec3 v_Id;

void main()
{
    FragColor = vec4(v_Id, 1.0);
}
