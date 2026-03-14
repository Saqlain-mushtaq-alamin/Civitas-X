#pragma once

#include <glm/vec2.hpp>

namespace civitasx
{

    namespace agents
    {

        enum class NpcState : int
        {
            Sleeping = 0,
            Working = 1,
            Walking = 2,
            Traveling = 3,
        };

        struct NpcAgent
        {
            int id = 0;
            glm::vec2 position{0.0f, 0.0f};
            int money = 100;
            NpcState state = NpcState::Sleeping;

            glm::vec2 home{0.0f, 0.0f};
            glm::vec2 work{0.0f, 0.0f};
            glm::vec2 food{0.0f, 0.0f};
            glm::vec2 target{0.0f, 0.0f};
            glm::vec2 accessAnchor{0.0f, 0.0f};

            int cycleStage = 0;
            float dwellSeconds = 0.0f;
            bool finalApproach = false;
            bool hasAccessAnchor = false;
        };

        NpcAgent makeDefaultNpc(int id, const glm::vec2 &spawnPosition);

    } // namespace agents

} // namespace civitasx
