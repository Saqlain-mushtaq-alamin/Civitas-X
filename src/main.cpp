#include "civitasx/app.hpp"

int main(int argc, char **argv)
{
    civitasx::App app;
    if (!app.initialize(argc, argv))
    {
        return 1;
    }

    app.run();
    return 0;
}
