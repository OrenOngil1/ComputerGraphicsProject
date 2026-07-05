#shader vertex
#version 330 core

// Sky pass: the cube is drawn with the view's translation stripped, so the
// viewer sits at its center and the corner position doubles as the cubemap
// sample direction.
layout(location = 0) in vec3 a_Position;

out vec3 v_Direction;

uniform mat4 u_ViewProj;   // projection * rotation-only view

void main()
{
    v_Direction = a_Position;
    vec4 pos = u_ViewProj * vec4(a_Position, 1.0);
    // z = w makes the depth 1.0 after the perspective divide: under the
    // app's GL_LEQUAL depth test the sky fills exactly the pixels the
    // terrain left untouched.
    gl_Position = pos.xyww;
}

#shader fragment
#version 330 core

layout(location = 0) out vec4 FragColor;

in vec3 v_Direction;

uniform samplerCube u_Skybox;

// Tilt every sample direction upward, sinking the apparent horizon (and this
// asset set's baked water-mirror line) behind the terrain. 0 = as authored;
// toward 1 = horizon sinks further. Applied AFTER normalizing, so the tilt
// depends only on the view direction, never on where the cube wall is --
// offsetting the unnormalized corner vector instead makes the box's edges
// show as creases (the offset's angular effect would vary with wall distance).
const float kEyeHeight = 0.22;

void main()
{
    vec3 dir = normalize(v_Direction);
    dir.y += kEyeHeight;
    FragColor = vec4(texture(u_Skybox, dir).rgb, 1.0);
}
