#shader vertex
#version 330 core

// Point-marker shader: picked correspondence points, recorded waypoints, the
// estimated-camera marker -- anything drawn as GL_POINTS. Identical vertex stage
// to the terrain shader (position + color through u_MVP); the fragment stage is
// where it differs: it carves the square point sprite into a disc.
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
    // gl_PointCoord runs (0,0)->(1,1) across the square point sprite. Keep only
    // the inscribed disc (radius 0.5, so r^2 = 0.25) and discard the corners.
    // This is the Core-profile stand-in for the deprecated GL_POINT_SMOOTH that
    // gave the legacy build round, square-free markers. gl_PointCoord is only
    // defined while rasterizing GL_POINTS, which is the sole use of this shader.
    vec2 d = gl_PointCoord - vec2(0.5);
    if (dot(d, d) > 0.25)
        discard;

    FragColor = vec4(v_Color, 1.0);
}
