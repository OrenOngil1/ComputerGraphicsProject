// Headless sanity checks for the GL-free math: the PnP solvers, the tracker
// blob centroids, the terrain normals, and the camera controls, each on
// synthetic inputs with a known correct answer. No window, no GL context --
// run anywhere with `ctest`.

#include <iostream>

#include "check.h"

void testNormals();
void testCentroids();
void testPnp();
void testCameraControls();

int main()
{
    testNormals();
    testCentroids();
    testPnp();
    testCameraControls();

    if (failures == 0)
        std::cout << "All checks passed." << std::endl;
    else
        std::cout << failures << " check(s) FAILED." << std::endl;
    return failures;
}
