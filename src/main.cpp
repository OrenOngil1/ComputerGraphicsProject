#include <cstdlib>
#include <exception>
#include <iostream>

#include "core/Application.h"

// The entire program is one Application: it owns the window, renderer, and app
// state, and run() drives the menu/session loop. All the bring-up and teardown
// ordering that used to live here is now encoded in Application's member layout.
//
// The try/catch is the one error boundary: lower layers (e.g. the Window ctor)
// throw a self-describing exception for any fatal bring-up failure, and it is
// logged exactly here, once, with a clean EXIT_FAILURE -- rather than letting it
// escape main() into std::terminate/abort, which would dump a less friendly
// message and skip stack unwinding.
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
