#pragma once

// The window rectangle a view paints into -- pure layout, no camera, no GL.
// Recomputed on resize (see leftHalf/rightHalf below).
struct Viewport {
    int x = 0;        // lower-left corner + size, in framebuffer PIXELS
    int y = 0;
    int width = 0;
    int height = 0;

    // Hit-test a pixel. Half-open on the far edges, so a pixel never counts as
    // inside two abutting viewports (the split shares the column at x + width).
    // Cursor coords arrive as doubles from GLFW, hence the parameter type.
    bool contains(double px, double py) const
    {
        return px >= x && px < x + width &&
               py >= y && py < y + height;
    }

    // Width-to-height ratio -- the factor mapping the vertical FOV to the
    // horizontal one in projections and viewing rays.
    float aspect() const { return (float)width / (float)height; }
};

// The split-screen layout as pure functions of the framebuffer size;
// `inline` because this header is widely included.
inline Viewport leftHalf(int windowWidth, int windowHeight)
{
    return Viewport{ 0, 0, windowWidth / 2, windowHeight };
}

inline Viewport rightHalf(int windowWidth, int windowHeight)
{
    return Viewport{ windowWidth / 2, 0, windowWidth / 2, windowHeight };
}
