#pragma once

#include <glm/vec2.hpp>

namespace civitasx
{

    namespace agents
    {

        struct NpcAgent
        {
            int id = 0;
            glm::vec2 position{0.0f, 0.0f};
            float mood = 1.0f;
        };

        NpcAgent makeDefaultNpc(int id, const glm::vec2 &spawnPosition);

    } // namespace agents

} // namespace civitasx
