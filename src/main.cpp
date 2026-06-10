#include "core/Application.h"

// The entire program is one Application: it owns the window, renderer, and app
// state, and run() drives the menu/session loop. All the bring-up and teardown
// ordering that used to live here is now encoded in Application's member layout.
int main()
{
    Application app;
    return app.run();
}
