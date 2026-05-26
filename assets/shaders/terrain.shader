#shader vertex
#version 330 core

// Shared shader for terrain and overlays. The `#shader vertex` / `#shader
// fragment` markers above are NOT standard GLSL -- they tell BasicOpenGL's
// Shader::ParseShader where to split this file into two GLSL programs.
//
// Per-vertex attributes -- locations match the order VertexBufferLayout pushes
// them in uploadTerrain (and drawPoints): 0 = position (3 floats), 1 = color.
layout(location = 0) in vec3 a_Position;
layout(location = 1) in vec3 a_Color;

// Pass-through to the fragment shader. Rasterization interpolates this value
// across the triangle, so a fragment sees a blend of its three corner colors.
out vec3 v_Color;

// Same for every vertex in a draw call -- set once via SetUniformMat4f.
uniform mat4 u_MVP;

void main()
{
    // gl_Position is the special output that puts this vertex in clip space.
    gl_Position = u_MVP * vec4(a_Position, 1.0);
    v_Color = a_Color;
}

#shader fragment
#version 330 core

// Final pixel color written to the framebuffer.
layout(location = 0) out vec4 FragColor;

// Interpolated from the vertex shader -- one value per rasterized pixel.
in vec3 v_Color;

void main()
{
    FragColor = vec4(v_Color, 1.0);
}
