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
            InCar = 4,
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

            // Car rental logic
            bool isRentingCar = false;
            int rentedCarId = -1;
        };

        // Update function for simulation
        void updateNpcAgent(NpcAgent &npc, float deltaSeconds, const glm::vec2 &destination, bool farDestination);

        NpcAgent makeDefaultNpc(int id, const glm::vec2 &spawnPosition);

    } // namespace agents

} // namespace civitasx
