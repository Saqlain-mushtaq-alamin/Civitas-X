#pragma once

namespace civitasx
{

    namespace systems
    {
        class SimulationManager;
    }

    namespace engine
    {

        class Renderer
        {
        public:
            void render(const systems::SimulationManager &simulation, int viewportWidth, int viewportHeight) const;
        };

    } // namespace engine

} // namespace civitasx
