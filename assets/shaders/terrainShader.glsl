#shader vertex
#version 330 core

// Shared shader for terrain and overlays. The `#shader vertex` / `#shader
// fragment` markers above are NOT standard GLSL -- they tell BasicOpenGL's
// Shader::ParseShader where to split this file into two GLSL programs.
//
// Per-vertex attributes -- locations match the order VertexBufferLayout pushes
// them in uploadBuffers (GpuMesh.cpp): 0 = position, 1 = color, 2 = normal
// (3, the terrain's pick-id color, belongs to the pick shader -- unused here).
// Draws whose VAO has no normal attribute (the overlays) leave a_Normal at the
// GL default -- harmless, they never take the lit path (u_Lit stays false).
layout(location = 0) in vec3 a_Position;
layout(location = 1) in vec3 a_Color;
layout(location = 2) in vec3 a_Normal;

// Pass-throughs to the fragment shader. Rasterization interpolates them
// across the triangle, so a fragment sees a blend of its three corner values.
out vec3 v_Color;
out vec3 v_Normal;

// Same for every vertex in a draw call -- set once via SetUniformMat4f.
uniform mat4 u_MVP;

void main()
{
    // gl_Position is the special output that puts this vertex in clip space.
    gl_Position = u_MVP * vec4(a_Position, 1.0);
    v_Color = a_Color;
    // Already world space: the terrain's only model transform is the baked
    // centering translation, and translations don't change normals.
    v_Normal = a_Normal;
}

#shader fragment
#version 330 core

// Final pixel color written to the framebuffer.
layout(location = 0) out vec4 FragColor;

// Interpolated from the vertex shader -- one value per rasterized pixel.
in vec3 v_Color;
in vec3 v_Normal;

// Ghost overlay path: when u_UseOverride is set, paint the whole mesh in one
// translucent color (rgb + alpha) instead of its per-vertex colors. Normal draws
// leave it false (uniforms default to 0/false), so terrain renders as before.
uniform bool u_UseOverride;
uniform vec4 u_OverrideColor;   // rgb tint + alpha; used only when u_UseOverride
uniform float u_TintStrength;   // 0 = keep terrain colors, 1 = flat override fill

// Directional light (Lambert + ambient), applied only while u_Lit is set --
// the renderer raises it around the terrain scene pass and the visible-frame
// tracker draw, dropping it after each, so every other draw through this same
// program (paths, ghost, capture passes) keeps its exact colors: pose markers
// must stay identifiable and the tracker / pick read-backs depend on unshaded
// values. vec4s because the Shader wrapper exposes no vec3 setter; the spare
// components are ignored.
uniform bool u_Lit;
uniform vec4 u_LightDir;     // xyz: world-space direction the light TRAVELS
uniform vec4 u_LightColor;   // rgb: diffuse light color
uniform float u_Ambient;     // illumination floor, so shadows aren't black

void main()
{
    vec3 color;
    float alpha;
    if (u_UseOverride) {
        // Ghost / tracker overlay: tint the mesh's own colors toward the override
        // hue -- not necessarily a flat color fill. u_TintStrength is independent
        // of alpha: it shapes the surface color, alpha controls how the result
        // blends over the framebuffer.
        color = mix(v_Color, u_OverrideColor.rgb, u_TintStrength);
        alpha = u_OverrideColor.a;
    } else {
        color = v_Color;
        alpha = 1.0;
    }

    if (u_Lit) {
        // Lambert's cosine law: a surface facing the light receives its
        // full color; one tilted away receives it scaled by the cosine of
        // the angle between normal and light. Interpolation can shorten
        // the normal, so renormalize per fragment.
        vec3 N = normalize(v_Normal);
        vec3 L = normalize(-u_LightDir.xyz);   // direction TOWARD the light
        float diffuse = max(dot(N, L), 0.0);
        color *= u_Ambient + diffuse * u_LightColor.rgb;
    }

    FragColor = vec4(color, alpha);
}
