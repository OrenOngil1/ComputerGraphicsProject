// Headless sanity checks for PoseLog -- the review cursor and the pose-error
// measure the console reports through it. No window, no GL context.

#include <cmath>
#include <iostream>

#include "check.h"
#include "../src/core/PoseLog.h"

// Entries only need to exist; the poses themselves are inert to the cursor.
static PoseEntry entry()
{
    return { Waypoint{ glm::vec3(0.0f), glm::vec3(0.0f) }, std::nullopt };
}

void testPoseLog()
{
    std::cout << "PoseLog:" << std::endl;

    PoseLog log;
    log.step(+1);
    log.step(-1);
    check(log.entries.empty() && log.current == 0, "step on an empty log is a no-op");

    log.add(entry());
    log.add(entry());
    log.add(entry());
    check(log.current == 2, "add: the cursor follows the newest capture");

    log.step(+1);
    check(log.current == 0, "step forward wraps from the last entry to the first");

    log.step(-1);
    check(log.current == 2, "step back wraps from the first entry to the last");

    log.step(-1);
    check(log.current == 1, "step back from the middle decrements");

    log.add(entry());
    check(log.current == 3, "add while reviewing: the cursor jumps to the newest");

    // ── poseError ─────────────────────────────────────────────
    std::cout << "poseError:" << std::endl;

    const Waypoint truth{ { 10, 20, 30 }, { 10, 20, 0 } };   // looking along -z

    const PoseError exact = poseError(truth, truth);
    check(exact.positionUnits == 0.0f && exact.headingDegrees == 0.0f,
          "a pose compared with itself is error-free in both halves");

    // Position: the distance between the eyes, whatever the targets are.
    const Waypoint shifted{ { 13, 24, 30 }, { -50, 0, -70 } };   // 3-4-5 away
    check(std::fabs(poseError(truth, shifted).positionUnits - 5.0f) < 1e-4f,
          "position error is the distance between the two eyes");

    // Heading: the case position alone declares perfect. Same eye, view swung
    // 30 degrees -- reported as 30 degrees off with zero position error.
    const float angle = glm::radians(30.0f);
    const Waypoint turned{ truth.position,
                           truth.position + glm::vec3(std::sin(angle), 0.0f,
                                                      -std::cos(angle)) * 30.0f };
    const PoseError swung = poseError(truth, turned);
    check(swung.positionUnits == 0.0f && std::fabs(swung.headingDegrees - 30.0f) < 1e-3f,
          "a camera in the right place looking 30 deg off reports 30 deg, not success");

    // Target distance must not leak into the angle: only the direction counts.
    const Waypoint far{ truth.position, truth.position + glm::vec3(0, 0, -900) };
    check(poseError(truth, far).headingDegrees < 1e-3f,
          "how far away the target sits does not change the heading error");

    // Degenerate pose (eye on its own target): no direction to compare, and the
    // position half must still be reported rather than swallowed by a NaN.
    const Waypoint degenerate{ { 13, 24, 30 }, { 13, 24, 30 } };
    const PoseError none = poseError(truth, degenerate);
    check(std::fabs(none.positionUnits - 5.0f) < 1e-4f && none.headingDegrees == 0.0f,
          "a directionless pose still reports its position error");
}
