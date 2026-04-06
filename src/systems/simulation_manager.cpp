#include "systems/simulation_manager.h"

#include <algorithm>
#include <limits>
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
            pendingRideRequests_.clear();

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
                npc.state = agents::NpcState::Idle;
                npc.cycleStage = 0;
                npc.dwellSeconds = 4.0f + static_cast<float>(i % 4);
                npc.assignedCarId = -1;
                npc.rideRequested = false;
                npcs_.push_back(npc);
            }
        }

        void SimulationManager::update(float deltaSeconds)
        {
            const float safeDelta = std::max(0.0f, deltaSeconds);
            const float pickupDistanceThreshold = 1.75f;
            const float arrivalDistanceThreshold = 1.0f;
            const float walkSpeed = 11.0f;
            const float farDestinationThreshold = 120.0f;

            auto findNpcById = [&](int npcId) -> agents::NpcAgent *
            {
                auto it = std::find_if(npcs_.begin(), npcs_.end(), [&](const agents::NpcAgent &npc)
                                       { return npc.id == npcId; });
                return (it != npcs_.end()) ? &(*it) : nullptr;
            };

            auto findCarById = [&](int carId) -> agents::CarAgent *
            {
                auto it = std::find_if(cars_.begin(), cars_.end(), [&](const agents::CarAgent &car)
                                       { return car.id == carId; });
                return (it != cars_.end()) ? &(*it) : nullptr;
            };

            auto pickDestinationForNpc = [](const agents::NpcAgent &npc) -> glm::vec2
            {
                if (npc.cycleStage == 0)
                {
                    return npc.work;
                }
                if (npc.cycleStage == 1)
                {
                    return npc.food;
                }
                return npc.home;
            };

            // 1) Handle new ride requests.
            std::vector<int> nextPendingRequests;
            nextPendingRequests.reserve(pendingRideRequests_.size());
            for (int npcId : pendingRideRequests_)
            {
                agents::NpcAgent *npc = findNpcById(npcId);
                if (npc == nullptr)
                {
                    continue;
                }

                // Ignore stale requests.
                if (npc->state != agents::NpcState::WaitingForCar || npc->assignedCarId >= 0)
                {
                    npc->rideRequested = false;
                    continue;
                }

                int bestCarIndex = -1;
                float bestDistance = std::numeric_limits<float>::max();
                for (std::size_t i = 0; i < cars_.size(); ++i)
                {
                    const agents::CarAgent &candidate = cars_[i];
                    if (candidate.state != agents::CarState::Free || candidate.isFueling)
                    {
                        continue;
                    }

                    const float distance = glm::distance(candidate.position, npc->position);
                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        bestCarIndex = static_cast<int>(i);
                    }
                }

                if (bestCarIndex < 0)
                {
                    nextPendingRequests.push_back(npcId);
                    continue;
                }

                agents::CarAgent &car = cars_[static_cast<std::size_t>(bestCarIndex)];
                car.state = agents::CarState::GoToPickup;
                car.passengerNpcId = npc->id;
                car.renterNpcId = npc->id;
                car.isRented = true;
                car.pickupLocation = npc->position;
                car.destination = npc->target;
                car.target = car.pickupLocation;

                npc->assignedCarId = car.id;
                npc->rideRequested = false;
                npc->state = agents::NpcState::WaitingForCar;
            }
            pendingRideRequests_.swap(nextPendingRequests);

            // 2) Assign cars and update cars.
            for (agents::CarAgent &car : cars_)
            {
                car.speed = ai::desiredSpeedForCar(car);

                float distance = 0.0f;
                switch (car.state)
                {
                case agents::CarState::Free:
                    if (car.position == car.target)
                    {
                        car.target = ai::chooseNextWaypoint(rng_, waypoints_, car.position);
                    }
                    distance = advanceCar(car, safeDelta);
                    if (car.position == car.target)
                    {
                        car.target = ai::chooseNextWaypoint(rng_, waypoints_, car.position);
                    }
                    break;
                case agents::CarState::Assigned:
                    car.state = agents::CarState::GoToPickup;
                    break;
                case agents::CarState::GoToPickup:
                    car.target = car.pickupLocation;
                    distance = advanceCar(car, safeDelta);
                    if (glm::distance(car.position, car.pickupLocation) <= pickupDistanceThreshold)
                    {
                        car.position = car.pickupLocation;
                        car.state = agents::CarState::WaitForNpc;
                    }
                    break;
                case agents::CarState::WaitForNpc:
                    // Car waits until NPC enters.
                    break;
                case agents::CarState::Transporting:
                    car.target = car.destination;
                    distance = advanceCar(car, safeDelta);
                    if (agents::NpcAgent *passenger = findNpcById(car.passengerNpcId))
                    {
                        passenger->position = car.position;
                    }
                    if (glm::distance(car.position, car.destination) <= arrivalDistanceThreshold)
                    {
                        car.position = car.destination;
                        if (agents::NpcAgent *passenger = findNpcById(car.passengerNpcId))
                        {
                            passenger->position = car.destination;
                            passenger->state = agents::NpcState::Arrived;
                            passenger->assignedCarId = -1;
                            passenger->isRentingCar = false;
                            passenger->rentedCarId = -1;
                        }

                        car.state = agents::CarState::Free;
                        car.passengerNpcId = -1;
                        car.renterNpcId = -1;
                        car.isRented = false;
                        car.destination = car.position;
                        car.pickupLocation = car.position;
                        car.target = ai::chooseNextWaypoint(rng_, waypoints_, car.position);
                    }
                    break;
                }

                // Car rental/fueling logic
                agents::updateCarAgent(car, safeDelta, distance);
                // Economy system: operational costs
                applyOperationalCosts(car, distance);

                // Fail-safe: fueling car cannot keep assignments.
                if (car.isFueling && car.state != agents::CarState::Free)
                {
                    if (agents::NpcAgent *passenger = findNpcById(car.passengerNpcId))
                    {
                        passenger->assignedCarId = -1;
                        if (passenger->state == agents::NpcState::InCar)
                        {
                            passenger->state = agents::NpcState::WaitingForCar;
                        }
                        if (!passenger->rideRequested)
                        {
                            pendingRideRequests_.push_back(passenger->id);
                            passenger->rideRequested = true;
                        }
                    }
                    car.state = agents::CarState::Free;
                    car.passengerNpcId = -1;
                    car.renterNpcId = -1;
                    car.isRented = false;
                    car.target = ai::chooseNextWaypoint(rng_, waypoints_, car.position);
                }
            }

            // 3) Update NPCs.
            for (agents::NpcAgent &npc : npcs_)
            {
                if (npc.dwellSeconds > 0.0f)
                {
                    npc.dwellSeconds -= safeDelta;
                    if (npc.dwellSeconds > 0.0f)
                    {
                        npc.state = agents::NpcState::Idle;
                        continue;
                    }
                    npc.dwellSeconds = 0.0f;
                }

                if (npc.state == agents::NpcState::WaitingForCar)
                {
                    if (npc.assignedCarId < 0)
                    {
                        if (!npc.rideRequested)
                        {
                            pendingRideRequests_.push_back(npc.id);
                            npc.rideRequested = true;
                        }
                        continue;
                    }

                    agents::CarAgent *assignedCar = findCarById(npc.assignedCarId);
                    if (assignedCar == nullptr)
                    {
                        npc.assignedCarId = -1;
                        if (!npc.rideRequested)
                        {
                            pendingRideRequests_.push_back(npc.id);
                            npc.rideRequested = true;
                        }
                        continue;
                    }

                    const bool carReadyForPickup =
                        assignedCar->state == agents::CarState::WaitForNpc &&
                        glm::distance(assignedCar->position, npc.position) <= pickupDistanceThreshold;

                    if (carReadyForPickup)
                    {
                        npc.state = agents::NpcState::InCar;
                        npc.position = assignedCar->position;
                        npc.isRentingCar = true;
                        npc.rentedCarId = assignedCar->id;

                        assignedCar->state = agents::CarState::Transporting;
                        assignedCar->passengerNpcId = npc.id;
                        assignedCar->destination = npc.target;
                    }
                    continue;
                }

                if (npc.state == agents::NpcState::InCar)
                {
                    agents::CarAgent *car = findCarById(npc.assignedCarId);
                    if (car != nullptr)
                    {
                        npc.position = car->position;
                    }
                    continue;
                }

                if (npc.state == agents::NpcState::Idle)
                {
                    npc.state = agents::NpcState::DecideDestination;
                }

                if (npc.state == agents::NpcState::DecideDestination)
                {
                    npc.target = pickDestinationForNpc(npc);
                    const float targetDistance = glm::distance(npc.position, npc.target);
                    if (targetDistance <= arrivalDistanceThreshold)
                    {
                        npc.state = agents::NpcState::Arrived;
                        continue;
                    }

                    const bool destinationIsFar = targetDistance > farDestinationThreshold;
                    if (destinationIsFar)
                    {
                        npc.state = agents::NpcState::RequestCar;
                    }
                    else
                    {
                        npc.state = agents::NpcState::Walking;
                    }
                }

                if (npc.state == agents::NpcState::RequestCar)
                {
                    if (!npc.rideRequested)
                    {
                        pendingRideRequests_.push_back(npc.id);
                        npc.rideRequested = true;
                    }
                    npc.state = agents::NpcState::WaitingForCar;
                    npc.assignedCarId = -1;
                    continue;
                }

                if (npc.state == agents::NpcState::Walking)
                {
                    const glm::vec2 toTarget = npc.target - npc.position;
                    const float distance = glm::length(toTarget);
                    if (distance <= arrivalDistanceThreshold)
                    {
                        npc.position = npc.target;
                        npc.state = agents::NpcState::Arrived;
                    }
                    else
                    {
                        const float step = std::min(walkSpeed * safeDelta, distance);
                        const glm::vec2 direction = toTarget / distance;
                        npc.position += direction * step;
                    }
                    continue;
                }

                if (npc.state == agents::NpcState::Arrived)
                {
                    if (npc.cycleStage == 0)
                    {
                        npc.money += 25.0f;
                    }
                    else if (npc.cycleStage == 1)
                    {
                        npc.money = std::max(0.0f, npc.money - 10.0f);
                    }

                    npc.cycleStage = (npc.cycleStage + 1) % 3;
                    npc.dwellSeconds = 3.5f + static_cast<float>(npc.id % 3);
                    npc.state = agents::NpcState::Idle;
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
