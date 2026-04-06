
#include <glm/glm.hpp>
#include <glm/geometric.hpp>
#include "agents/npc_agent.h"

namespace civitasx
{

    namespace agents
    {

        NpcAgent makeDefaultNpc(int id, const glm::vec2 &spawnPosition)
        {
            NpcAgent npc;
            npc.id = id;
            npc.position = spawnPosition;
            npc.money = 100;
            npc.state = NpcState::Idle;
            npc.home = spawnPosition;
            npc.work = spawnPosition;
            npc.food = spawnPosition;
            npc.target = spawnPosition;
            npc.accessAnchor = spawnPosition;
            npc.cycleStage = 0;
            npc.dwellSeconds = 0.0f;
            npc.finalApproach = false;
            npc.hasAccessAnchor = false;
            npc.assignedCarId = -1;
            npc.rideRequested = false;
            return npc;
        }

        void updateNpcAgent(NpcAgent &npc, float deltaSeconds, const glm::vec2 &destination, bool farDestination)
        {
            // If destination is far and not already renting a car, rent one
            if (farDestination && !npc.isRentingCar)
            {
                // In a real system, find an available car and set rentedCarId
                npc.isRentingCar = true;
                npc.rentedCarId = 0; // Example: always rent car 0
                npc.state = NpcState::InCar;
                npc.money -= 5; // Pay rent upfront
            }

            // If renting a car, move with car and pay per frame
            if (npc.isRentingCar)
            {
                // In a real system, sync position with car
                npc.money -= 0.1f * deltaSeconds; // Pay per second
                // If arrived at destination, stop renting
                if (glm::distance(npc.position, destination) < 1.0f)
                {
                    npc.isRentingCar = false;
                    npc.rentedCarId = -1;
                    npc.state = NpcState::Walking;
                }
            }
            else
            {
                // Normal walking/traveling logic
                if (glm::distance(npc.position, destination) > 1.0f)
                {
                    glm::vec2 dir = glm::normalize(destination - npc.position);
                    npc.position += dir * 2.0f * deltaSeconds; // Walk speed
                    npc.state = NpcState::Walking;
                }
                else
                {
                    npc.state = NpcState::Sleeping; // Example: arrived
                }
            }
        }

    } // namespace agents

} // namespace civitasx
