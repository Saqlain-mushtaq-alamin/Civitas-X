#include "systems/simulation_manager.h"

#include <algorithm>
#include <random>

#include <glm/geometric.hpp>

#include "ai/behavior_system.h"
#include "ai/pathfinding.h"
#include "systems/economy_system.h"
#include "systems/traffic_system.h"
#include "world/road_network.h"
#include "world/tile_system.h"

namespace civitasx
{

    namespace systems
    {

        void SimulationManager::initialize(unsigned int seed, int carCount)
        {
            rng_.seed(seed);

            cityMap_.initializeDefault();

            world::RoadNetwork roads;
            waypoints_ = roads.buildWaypoints(cityMap_.config());

            world::TileSystem tiles;
            for (glm::vec2 &point : waypoints_)
            {
                point = tiles.snapToGrid(point, cityMap_.config().tileSize);
            }

            cars_.clear();
            npcs_.clear();

            if (waypoints_.empty())
            {
                return;
            }

            std::uniform_int_distribution<std::size_t> pointDist(0, waypoints_.size() - 1U);
            std::uniform_real_distribution<float> speedDist(18.0f, 34.0f);

            for (int i = 0; i < carCount; ++i)
            {
                const glm::vec2 spawn = waypoints_[pointDist(rng_)];
                agents::CarAgent car = agents::makeDefaultCar(i, spawn, speedDist(rng_));
                car.target = ai::chooseNextWaypoint(rng_, waypoints_, car.position);
                cars_.push_back(car);
            }

            const int npcCount = std::max(10, carCount / 2);
            for (int i = 0; i < npcCount; ++i)
            {
                const glm::vec2 home = waypoints_[pointDist(rng_)];

                glm::vec2 work = waypoints_[pointDist(rng_)];
                int guard = 0;
                while (work == home && guard < 12)
                {
                    work = waypoints_[pointDist(rng_)];
                    ++guard;
                }

                glm::vec2 food = waypoints_[pointDist(rng_)];
                guard = 0;
                while ((food == home || food == work) && guard < 12)
                {
                    food = waypoints_[pointDist(rng_)];
                    ++guard;
                }

                agents::NpcAgent npc = agents::makeDefaultNpc(i, home);
                npc.home = home;
                npc.work = work;
                npc.food = food;
                npc.target = work;
                npc.money = 80 + ((i * 11) % 120);
                npc.state = agents::NpcState::Sleeping;
                npc.cycleStage = 0;
                npc.dwellSeconds = 4.0f + static_cast<float>(i % 4);
                npcs_.push_back(npc);
            }
        }

        void SimulationManager::update(float deltaSeconds)
        {
            const float safeDelta = std::max(0.0f, deltaSeconds);

            for (agents::CarAgent &car : cars_)
            {
                car.speed = ai::desiredSpeedForCar(car);

                if (car.position == car.target)
                {
                    car.target = ai::chooseNextWaypoint(rng_, waypoints_, car.position);
                }

                const float distance = advanceCar(car, deltaSeconds);
                applyOperationalCosts(car, distance);

                if (car.position == car.target)
                {
                    car.target = ai::chooseNextWaypoint(rng_, waypoints_, car.position);
                }
            }

            const float travelSpeed = 26.0f;
            const float walkSpeed = 11.0f;

            for (agents::NpcAgent &npc : npcs_)
            {
                if (npc.dwellSeconds > 0.0f)
                {
                    npc.dwellSeconds -= safeDelta;
                    if (npc.dwellSeconds < 0.0f)
                    {
                        npc.dwellSeconds = 0.0f;
                    }

                    if (npc.cycleStage == 0)
                    {
                        npc.state = agents::NpcState::Sleeping;
                    }
                    else if (npc.cycleStage == 1)
                    {
                        npc.state = agents::NpcState::Working;
                    }
                    else
                    {
                        npc.state = agents::NpcState::Walking;
                    }
                    continue;
                }

                const glm::vec2 toTarget = npc.target - npc.position;
                const float distance = glm::length(toTarget);

                if (distance <= 0.75f)
                {
                    npc.position = npc.target;

                    if (npc.cycleStage == 0)
                    {
                        // Arrived at work.
                        npc.state = agents::NpcState::Working;
                        npc.money += 25;
                        npc.dwellSeconds = 12.0f;
                        npc.target = npc.food;
                        npc.cycleStage = 1;
                    }
                    else if (npc.cycleStage == 1)
                    {
                        // Arrived at food location.
                        npc.state = agents::NpcState::Walking;
                        npc.money = std::max(0, npc.money - 10);
                        npc.dwellSeconds = 7.0f;
                        npc.target = npc.home;
                        npc.cycleStage = 2;
                    }
                    else
                    {
                        // Arrived back home.
                        npc.state = agents::NpcState::Sleeping;
                        npc.dwellSeconds = 14.0f;
                        npc.target = npc.work;
                        npc.cycleStage = 0;
                    }

                    continue;
                }

                const glm::vec2 direction = toTarget / distance;
                const float speed = (distance < 16.0f) ? walkSpeed : travelSpeed;
                const float step = std::min(speed * safeDelta, distance);
                npc.position += direction * step;
                npc.state = (speed == walkSpeed) ? agents::NpcState::Walking : agents::NpcState::Traveling;
            }
        }

        const std::vector<agents::CarAgent> &SimulationManager::cars() const
        {
            return cars_;
        }

        const std::vector<agents::NpcAgent> &SimulationManager::npcs() const
        {
            return npcs_;
        }

        const std::vector<glm::vec2> &SimulationManager::waypoints() const
        {
            return waypoints_;
        }

        const world::CityMapConfig &SimulationManager::cityMap() const
        {
            return cityMap_.config();
        }

    } // namespace systems

} // namespace civitasx
