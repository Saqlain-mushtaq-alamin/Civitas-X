#pragma once

#include <glm/vec2.hpp>

namespace civitasx
{

    namespace agents
    {

        struct CarAgent
        {
            int id = 0;
            glm::vec2 position{0.0f, 0.0f};
            glm::vec2 target{0.0f, 0.0f};
            float speed = 25.0f;
            float battery = 100.0f;
            float wallet = 50.0f;
        };

        CarAgent makeDefaultCar(int id, const glm::vec2 &spawnPosition, float speed);

    } // namespace agents

} // namespace civitasx
