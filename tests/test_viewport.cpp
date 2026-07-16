// Headless sanity checks for Viewport hit-testing and the split-screen
// layout helpers -- no window, no GL context.

#include <cmath>
#include <iostream>

#include "check.h"
#include "../src/core/Viewport.h"

void testViewport()
{
    std::cout << "Viewport:" << std::endl;

    const Viewport vp{ 100, 0, 300, 200 };

    check(vp.contains(100, 0), "contains: the (x, y) corner is inside");
    check(vp.contains(399.9, 199.9), "contains: just inside the far edges");
    check(!vp.contains(400, 100), "contains: x + width is outside (half-open)");
    check(!vp.contains(250, 200), "contains: y + height is outside (half-open)");
    check(!vp.contains(99.9, 100), "contains: left of x is outside");
    check(std::abs(vp.aspect() - 1.5f) < 1e-6f, "aspect: width over height");

    // The split promise: the two halves tile the framebuffer -- every pixel in
    // exactly one -- with the seam column at width/2 owned by the right half.
    const int w = 800, h = 600;
    const Viewport left = leftHalf(w, h), right = rightHalf(w, h);

    check(!left.contains(w / 2, 0) && right.contains(w / 2, 0),
          "split: the seam column belongs to the right half only");
    check(left.contains(w / 2 - 1, 0) && !right.contains(w / 2 - 1, 0),
          "split: the column before the seam belongs to the left half only");

    bool exactlyOne = true;
    for (int x = 0; x < w && exactlyOne; x++)
        for (int y = 0; y < h; y += 100)
            exactlyOne = exactlyOne && (left.contains(x, y) != right.contains(x, y));
    check(exactlyOne, "split: every framebuffer column is in exactly one half");
}
