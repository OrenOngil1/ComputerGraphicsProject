#include "Simulation.h"

#include "../state/State.h"

// Defined here, where State is a complete type, so the unique_ptr<State> member
// can be destroyed -- it cannot be in a TU that only sees the forward declaration.
Simulation::~Simulation() = default;
