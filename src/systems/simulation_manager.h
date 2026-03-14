#pragma once

#include <random>
#include <vector>

#include <glm/vec2.hpp>

#include "agents/car_agent.h"
#include "agents/npc_agent.h"
#include "world/city_map.h"
#include "ai/pathfinding.h"

namespace civitasx
{

    namespace systems
    {

        class SimulationManager
        {
        public:
            void initialize(unsigned int seed = 7U, int carCount = 24);
            void update(float deltaSeconds);

            const std::vector<agents::CarAgent> &cars() const;
            const std::vector<agents::NpcAgent> &npcs() const;
            const std::vector<glm::vec2> &waypoints() const;
            const world::CityMapConfig &cityMap() const;

        private:
            std::mt19937 rng_;
            world::CityMap cityMap_;
            std::vector<glm::vec2> waypoints_;
            ai::RoadGraph roadGraph_;
            std::vector<agents::CarAgent> cars_;
            std::vector<agents::NpcAgent> npcs_;
        };

    } // namespace systems

} // namespace civitasx
