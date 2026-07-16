#shader vertex
#version 330 core

// Color-pick pass: render each triangle in a flat color that encodes the id of
// its provoking vertex, so reading one pixel back tells us which vertex was
// clicked. The encoding is CPU-side (PickEncoding.h) and arrives pre-baked as
// a per-vertex attribute -- this shader carries no packing rule of its own.
layout(location = 0) in vec3 a_Position;      // attributes 1-2 (color, normal) are ignored here
layout(location = 3) in vec3 a_IdColor;       // encodeVertexId(index), baked at terrain upload

// `flat` = no interpolation: every fragment of a triangle gets the provoking
// vertex's value verbatim, so it decodes to one whole vertex id (a smooth/
// interpolated varying would blend three ids into a meaningless color).
flat out vec3 v_Id;

uniform mat4 u_MVP;

void main()
{
    gl_Position = u_MVP * vec4(a_Position, 1.0);
    v_Id = a_IdColor;
}

#shader fragment
#version 330 core

layout(location = 0) out vec4 FragColor;

flat in vec3 v_Id;

void main()
{
    FragColor = vec4(v_Id, 1.0);
}
