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

            // Build road graph for pathfinding
            roadGraph_ = ai::buildRoadGraph(cityMap_, static_cast<int>(cityMap_.config().tileSize));

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

            // --- Update Cars ---
            for (agents::CarAgent &car : cars_)
            {
                car.speed = ai::desiredSpeedForCar(car);
                if (car.position == car.target)
                {
                    car.target = ai::chooseNextWaypoint(rng_, waypoints_, car.position);
                }
                float distance = advanceCar(car, deltaSeconds);
                // Car rental/fueling logic
                agents::updateCarAgent(car, deltaSeconds, distance);
                // Economy system: operational costs
                applyOperationalCosts(car, distance);
                if (car.position == car.target)
                {
                    car.target = ai::chooseNextWaypoint(rng_, waypoints_, car.position);
                }
            }

            const float travelSpeed = 26.0f;
            const float walkSpeed = 11.0f;

            // --- Update NPCs ---
            for (agents::NpcAgent &npc : npcs_)
            {
                // Handle dwell (sleep, work, eat)
                if (npc.dwellSeconds > 0.0f)
                {
                    npc.dwellSeconds -= safeDelta;
                    if (npc.dwellSeconds < 0.0f)
                        npc.dwellSeconds = 0.0f;
                    if (npc.cycleStage == 0)
                        npc.state = agents::NpcState::Sleeping;
                    else if (npc.cycleStage == 1)
                        npc.state = agents::NpcState::Working;
                    else
                        npc.state = agents::NpcState::Walking;
                    // Economy: earn money if working
                    if (npc.state == agents::NpcState::Working)
                        npc.money += 1;
                    continue;
                }

                // --- Pathfinding: Compute path if needed ---
                // If path is empty or target changed, compute new path
                if (npc.pathNodes.empty() || npc.currentPathIndex >= static_cast<int>(npc.pathNodes.size()) || glm::distance(npc.target, roadGraph_.nodeCenters[npc.pathNodes.back()]) > 1.0f)
                {
                    // Find closest road node to NPC and to target
                    int closestStart = -1, closestGoal = -1;
                    float bestStartDist = 1e9f, bestGoalDist = 1e9f;
                    for (int i = 0; i < static_cast<int>(roadGraph_.nodeCenters.size()); ++i)
                    {
                        float dStart = glm::distance(npc.position, roadGraph_.nodeCenters[i]);
                        float dGoal = glm::distance(npc.target, roadGraph_.nodeCenters[i]);
                        if (dStart < bestStartDist)
                        {
                            bestStartDist = dStart;
                            closestStart = i;
                        }
                        if (dGoal < bestGoalDist)
                        {
                            bestGoalDist = dGoal;
                            closestGoal = i;
                        }
                    }
                    if (closestStart >= 0 && closestGoal >= 0)
                    {
                        npc.pathNodes = ai::findPathAStar(roadGraph_, closestStart, closestGoal);
                        npc.currentPathIndex = 0;
                    }
                }

                // --- Car rental logic ---
                bool farDestination = false;
                if (!npc.pathNodes.empty() && npc.currentPathIndex < static_cast<int>(npc.pathNodes.size()))
                {
                    // If path is long, consider it far
                    float totalPathDist = 0.0f;
                    for (size_t i = 1; i < npc.pathNodes.size(); ++i)
                        totalPathDist += glm::distance(roadGraph_.nodeCenters[npc.pathNodes[i - 1]], roadGraph_.nodeCenters[npc.pathNodes[i]]);
                    farDestination = (totalPathDist > 16.0f);
                }
                if (farDestination && !npc.isRentingCar)
                {
                    // Find available car
                    int rentedCarId = -1;
                    for (auto &car : cars_)
                    {
                        if (!car.isRented && !car.isFueling)
                        {
                            car.isRented = true;
                            car.renterNpcId = npc.id;
                            rentedCarId = car.id;
                            break;
                        }
                    }
                    if (rentedCarId != -1 && npc.money >= 5)
                    {
                        npc.isRentingCar = true;
                        npc.rentedCarId = rentedCarId;
                        npc.state = agents::NpcState::InCar;
                        npc.money -= 5; // Pay rent upfront
                    }
                }

                // --- Move NPC along path ---
                if (npc.isRentingCar)
                {
                    // Move with car
                    auto carIt = std::find_if(cars_.begin(), cars_.end(), [&](const agents::CarAgent &car)
                                              { return car.id == npc.rentedCarId; });
                    if (carIt != cars_.end())
                    {
                        npc.position = carIt->position;
                        npc.money -= 0.1f * safeDelta;
                        // If car is fueling or out of battery, stop renting
                        if (carIt->isFueling || carIt->battery < 10.0f)
                        {
                            npc.isRentingCar = false;
                            npc.rentedCarId = -1;
                            npc.state = agents::NpcState::Walking;
                        }
                        // If arrived at destination, stop renting
                        else if (!npc.pathNodes.empty() && npc.currentPathIndex >= static_cast<int>(npc.pathNodes.size()) - 1 && glm::distance(npc.position, npc.target) < 1.0f)
                        {
                            npc.isRentingCar = false;
                            npc.rentedCarId = -1;
                            npc.state = agents::NpcState::Walking;
                        }
                        continue;
                    }
                    else
                    {
                        npc.isRentingCar = false;
                        npc.rentedCarId = -1;
                        npc.state = agents::NpcState::Walking;
                    }
                }

                // Walk/travel along path
                if (!npc.pathNodes.empty() && npc.currentPathIndex < static_cast<int>(npc.pathNodes.size()))
                {
                    glm::vec2 nextWaypoint = roadGraph_.nodeCenters[npc.pathNodes[npc.currentPathIndex]];
                    float distToWaypoint = glm::distance(npc.position, nextWaypoint);
                    float speed = (farDestination ? travelSpeed : walkSpeed);
                    float step = std::min(speed * safeDelta, distToWaypoint);
                    if (distToWaypoint <= 0.75f)
                    {
                        npc.position = nextWaypoint;
                        npc.currentPathIndex++;
                    }
                    else
                    {
                        glm::vec2 direction = (nextWaypoint - npc.position) / distToWaypoint;
                        npc.position += direction * step;
                        npc.state = (speed == walkSpeed) ? agents::NpcState::Walking : agents::NpcState::Traveling;
                    }

                    // If reached final waypoint and close to target, complete journey
                    if (npc.currentPathIndex >= static_cast<int>(npc.pathNodes.size()) && glm::distance(npc.position, npc.target) <= 0.75f)
                    {
                        npc.position = npc.target;
                        npc.pathNodes.clear();
                        npc.currentPathIndex = 0;
                        // Arrived at destination, handle cycle
                        if (npc.cycleStage == 0)
                        {
                            npc.state = agents::NpcState::Working;
                            npc.money += 25;
                            npc.dwellSeconds = 12.0f;
                            npc.target = npc.food;
                            npc.cycleStage = 1;
                        }
                        else if (npc.cycleStage == 1)
                        {
                            npc.state = agents::NpcState::Walking;
                            npc.money = std::max(0.0f, npc.money - 10.0f);
                            npc.dwellSeconds = 7.0f;
                            npc.target = npc.home;
                            npc.cycleStage = 2;
                        }
                        else
                        {
                            npc.state = agents::NpcState::Sleeping;
                            npc.dwellSeconds = 14.0f;
                            npc.target = npc.work;
                            npc.cycleStage = 0;
                        }
                    }
                }
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
