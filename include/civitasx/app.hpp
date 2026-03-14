#pragma once

#include "engine/renderer.h"
#include "systems/simulation_manager.h"

namespace civitasx
{

    class App
    {
    public:
        bool initialize(int &argc, char **argv);
        void run();

    private:
        static App *instance_;

        static void displayCallback();
        static void reshapeCallback(int width, int height);
        static void idleCallback();

        void onDisplay();
        void onReshape(int width, int height);
        void onIdle();

        int width_ = 1280;
        int height_ = 720;
        float lastTimeSeconds_ = 0.0f;
        systems::SimulationManager simulation_;
        engine::Renderer renderer_;
    };

} // namespace civitasx
