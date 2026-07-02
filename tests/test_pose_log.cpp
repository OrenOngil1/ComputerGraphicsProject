// Headless sanity checks for PoseLog's review cursor -- no window, no GL context.

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
}
