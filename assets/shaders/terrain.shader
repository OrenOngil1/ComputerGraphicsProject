#shader vertex
#version 330 core

layout(location = 0) in vec3 a_Position;
layout(location = 1) in vec3 a_Color;

out vec3 v_Color;

uniform mat4 u_MVP;

void main()
{
    gl_Position = u_MVP * vec4(a_Position, 1.0);
    v_Color = a_Color;
}

#shader fragment
#version 330 core

layout(location = 0) out vec4 FragColor;

in vec3 v_Color;

void main()
{
    FragColor = vec4(v_Color, 1.0);
}
