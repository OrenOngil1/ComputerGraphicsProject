#include <cstdlib>
#include <exception>
#include <iostream>

#include "core/Application.h"

// The one error boundary: lower layers (e.g. the Window ctor) throw a
// self-describing exception on any fatal bring-up failure; it is logged here,
// once, with a clean EXIT_FAILURE instead of escaping into std::terminate.
int main()
{
    try {
        Application app;
        return app.run();
    } catch (const std::exception &e) {
        std::cerr << "Fatal: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
