// Headless sanity checks for the GL-free pieces -- geometry math, vision
// steps, and the cross-layer contracts -- each on synthetic inputs with a
// known correct answer, one test_*.cpp per topic. No window, no GL context:
// run anywhere with `ctest`.

#include <iostream>

#include "check.h"

void testNormals();
void testCentroids();
void testPnp();
void testCameraControls();
void testPickEncoding();
void testCameraModel();
void testViewport();
void testPoseLog();
void testFeatures();
void testFeatureMatching();
void testFeatureDbIo();
void testSkyboxLoader();
void testViewAids();
void testRayAnchor();
void testManualBuild();

int main()
{
    testNormals();
    testCentroids();
    testPnp();
    testCameraControls();
    testPickEncoding();
    testCameraModel();
    testViewport();
    testPoseLog();
    testFeatures();
    testFeatureMatching();
    testFeatureDbIo();
    testSkyboxLoader();
    testViewAids();
    testRayAnchor();
    testManualBuild();

    if (failures == 0)
        std::cout << "All checks passed." << std::endl;
    else
        std::cout << failures << " check(s) FAILED." << std::endl;
    return failures;
}
