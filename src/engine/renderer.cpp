#include "engine/renderer.h"

#include <GL/freeglut.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <glm/geometric.hpp>

#include "agents/car_agent.h"
#include "agents/npc_agent.h"
#include "ai/pathfinding.h"
#include "engine/input.h"
#include "graphics/algorithms.h"
#include "systems/traffic_system.h"
#include "world/city_map.h"

namespace civitasx
{

    namespace engine
    {

        namespace
        {

            enum class BuildingStyle
            {
                House,
                Office,
            };

            bool isRoadTile(const world::CityMap &map, int row, int col)
            {
                if (row < 0 || col < 0)
                {
                    return false;
                }

                const std::size_t rowIndex = static_cast<std::size_t>(row);
                const std::size_t colIndex = static_cast<std::size_t>(col);
                if (rowIndex >= map.rows() || colIndex >= map.cols())
                {
                    return false;
                }

                return map.tileAt(rowIndex, colIndex) == world::TileType::Road;
            }

            BuildingStyle chooseBuildingStyle(const world::CityMap &map, int row, int col)
            {
                // Make offices appear around stronger road activity, houses elsewhere.
                const int roadNeighbors =
                    (isRoadTile(map, row, col - 1) ? 1 : 0) +
                    (isRoadTile(map, row, col + 1) ? 1 : 0) +
                    (isRoadTile(map, row - 1, col) ? 1 : 0) +
                    (isRoadTile(map, row + 1, col) ? 1 : 0);

                if (roadNeighbors >= 2)
                {
                    return BuildingStyle::Office;
                }

                // Deterministic variation for blocks not directly connected to major roads.
                if (((row * 13) + (col * 7)) % 5 == 0)
                {
                    return BuildingStyle::Office;
                }

                return BuildingStyle::House;
            }

            struct CarRuntimeState
            {
                bool initialized = false;
                float lastSimulationSeconds = 0.0f;
                std::mt19937 rng{17U};
                std::vector<agents::CarAgent> cars;
                std::vector<int> currentNodeByCar;
                std::vector<std::vector<int>> routeByCar;
                std::vector<std::size_t> routeStepByCar;
                ai::RoadGraph graph;
            };

            CarRuntimeState g_carRuntime;

            struct NpcRuntimeState
            {
                bool initialized = false;
                float lastSimulationSeconds = 0.0f;
                std::mt19937 rng{73U};
                std::vector<agents::NpcAgent> npcs;
            };

            NpcRuntimeState g_npcRuntime;

            void drawPoints(const std::vector<glm::ivec2> &points);

            glm::vec2 tileCenter(int row, int col, int tilePixels)
            {
                return {
                    static_cast<float>(col * tilePixels + (tilePixels / 2)),
                    static_cast<float>(row * tilePixels + (tilePixels / 2))};
            }

            bool assignRouteForCar(std::size_t carIndex, int startNode)
            {
                if (carIndex >= g_carRuntime.cars.size() ||
                    startNode < 0 ||
                    startNode >= static_cast<int>(g_carRuntime.graph.nodeCenters.size()))
                {
                    return false;
                }

                int goalNode = -1;
                if (!ai::chooseRandomGoalNode(g_carRuntime.rng, g_carRuntime.graph, startNode, goalNode))
                {
                    return false;
                }

                std::vector<int> path = ai::findPathAStar(g_carRuntime.graph, startNode, goalNode);
                if (path.size() < 2U)
                {
                    // Fallback if no path found: hop to any direct neighbor.
                    const std::vector<int> &neighbors = g_carRuntime.graph.adjacency[static_cast<std::size_t>(startNode)];
                    if (neighbors.empty())
                    {
                        return false;
                    }

                    std::uniform_int_distribution<std::size_t> pick(0, neighbors.size() - 1U);
                    path = {startNode, neighbors[pick(g_carRuntime.rng)]};
                }

                agents::CarAgent &car = g_carRuntime.cars[carIndex];
                g_carRuntime.currentNodeByCar[carIndex] = startNode;
                g_carRuntime.routeByCar[carIndex] = std::move(path);
                g_carRuntime.routeStepByCar[carIndex] = 0U;

                const int nextNode = g_carRuntime.routeByCar[carIndex][1];
                car.target = g_carRuntime.graph.nodeCenters[static_cast<std::size_t>(nextNode)];
                const glm::vec2 heading = car.target - car.position;
                if (glm::length(heading) > 0.0001f)
                {
                    car.angle = std::atan2(heading.y, heading.x);
                }

                return true;
            }

            void initializeCars(const world::CityMap &map, int tilePixels)
            {
                if (g_carRuntime.initialized)
                {
                    return;
                }

                g_carRuntime.graph = ai::buildRoadGraph(map, tilePixels);

                g_carRuntime.cars.clear();
                g_carRuntime.currentNodeByCar.clear();
                g_carRuntime.routeByCar.clear();
                g_carRuntime.routeStepByCar.clear();

                if (!g_carRuntime.graph.nodeCenters.empty())
                {
                    const int desiredCars = std::min(48, static_cast<int>(g_carRuntime.graph.nodeCenters.size()));
                    std::uniform_real_distribution<float> speedDist(32.0f, 62.0f);
                    std::uniform_int_distribution<std::size_t> startDist(0, g_carRuntime.graph.nodeCenters.size() - 1U);

                    for (int i = 0; i < desiredCars; ++i)
                    {
                        const int startNode = static_cast<int>(startDist(g_carRuntime.rng));
                        agents::CarAgent car = agents::makeDefaultCar(
                            i,
                            g_carRuntime.graph.nodeCenters[static_cast<std::size_t>(startNode)],
                            speedDist(g_carRuntime.rng));

                        g_carRuntime.cars.push_back(car);
                        g_carRuntime.currentNodeByCar.push_back(startNode);
                        g_carRuntime.routeByCar.push_back({});
                        g_carRuntime.routeStepByCar.push_back(0U);

                        assignRouteForCar(static_cast<std::size_t>(i), startNode);
                    }
                }

                g_carRuntime.initialized = true;
            }

            void updateCars(float deltaSeconds, const world::CityMap &map, int tilePixels, float simulationSeconds)
            {
                if (deltaSeconds <= 0.0f)
                {
                    return;
                }

                const float laneOffset = 2.2f;
                const float reachThreshold = 0.75f;
                for (std::size_t i = 0; i < g_carRuntime.cars.size(); ++i)
                {
                    agents::CarAgent &car = g_carRuntime.cars[i];

                    int currentNode = g_carRuntime.currentNodeByCar[i];
                    if (currentNode < 0 || currentNode >= static_cast<int>(g_carRuntime.graph.nodeCenters.size()))
                    {
                        continue;
                    }

                    std::vector<int> &route = g_carRuntime.routeByCar[i];
                    std::size_t &routeStep = g_carRuntime.routeStepByCar[i];
                    if (route.size() < 2U || routeStep + 1U >= route.size())
                    {
                        assignRouteForCar(i, currentNode);
                    }

                    if (route.size() < 2U || routeStep + 1U >= route.size())
                    {
                        continue;
                    }

                    int targetNode = route[routeStep + 1U];
                    car.target = g_carRuntime.graph.nodeCenters[static_cast<std::size_t>(targetNode)];

                    const glm::vec2 toTarget = car.target - car.position;
                    if (glm::length(toTarget) <= reachThreshold)
                    {
                        car.position = car.target;

                        currentNode = targetNode;
                        g_carRuntime.currentNodeByCar[i] = currentNode;
                        if (routeStep + 1U < route.size())
                        {
                            ++routeStep;
                        }

                        if (routeStep + 1U >= route.size())
                        {
                            assignRouteForCar(i, currentNode);
                            if (g_carRuntime.routeByCar[i].size() < 2U || g_carRuntime.routeStepByCar[i] + 1U >= g_carRuntime.routeByCar[i].size())
                            {
                                continue;
                            }
                            route = g_carRuntime.routeByCar[i];
                            routeStep = g_carRuntime.routeStepByCar[i];
                        }

                        targetNode = route[routeStep + 1U];
                        car.target = g_carRuntime.graph.nodeCenters[static_cast<std::size_t>(targetNode)];
                    }

                    const glm::vec2 toNext = car.target - car.position;
                    const float distanceToNext = glm::length(toNext);
                    if (distanceToNext <= 0.0001f)
                    {
                        continue;
                    }

                    const glm::vec2 direction = toNext / distanceToNext;
                    car.angle = std::atan2(direction.y, direction.x);

                    float speedScale = 1.0f;
                    float maxTravel = distanceToNext;

                    const bool hasValidTargetNode =
                        (targetNode >= 0 && targetNode < static_cast<int>(g_carRuntime.graph.nodeCenters.size()));
                    const bool hasValidCurrentNode =
                        (currentNode >= 0 && currentNode < static_cast<int>(g_carRuntime.graph.nodeCenters.size()));

                    // Once a car enters an intersection box, keep it flowing through.
                    bool committedInIntersection = false;
                    if (hasValidCurrentNode)
                    {
                        const int currentRow = g_carRuntime.graph.nodeRowsById[static_cast<std::size_t>(currentNode)];
                        const int currentCol = g_carRuntime.graph.nodeColsById[static_cast<std::size_t>(currentNode)];
                        if (systems::isSignalizedIntersection(map, currentRow, currentCol))
                        {
                            const float distanceFromCurrentCenter =
                                glm::length(car.position - g_carRuntime.graph.nodeCenters[static_cast<std::size_t>(currentNode)]);
                            committedInIntersection = (distanceFromCurrentCenter <= (static_cast<float>(tilePixels) * 0.7f));
                        }
                    }

                    if (hasValidTargetNode && !committedInIntersection)
                    {
                        const int targetRow = g_carRuntime.graph.nodeRowsById[static_cast<std::size_t>(targetNode)];
                        const int targetCol = g_carRuntime.graph.nodeColsById[static_cast<std::size_t>(targetNode)];

                        if (systems::isSignalizedIntersection(map, targetRow, targetCol))
                        {
                            const systems::IntersectionSignalState signal =
                                systems::queryIntersectionSignal(map, targetRow, targetCol, simulationSeconds);

                            const bool horizontalMovement = std::fabs(direction.x) >= std::fabs(direction.y);
                            const systems::TrafficLightColor activeColor =
                                horizontalMovement ? signal.horizontal : signal.vertical;

                            const float stopDistance = static_cast<float>(tilePixels) * 0.43f;

                            if (activeColor == systems::TrafficLightColor::Red)
                            {
                                speedScale = 0.0f;
                                maxTravel = std::max(0.0f, distanceToNext - stopDistance);
                            }
                            else if (activeColor == systems::TrafficLightColor::Yellow)
                            {
                                speedScale = 0.35f;
                                maxTravel = std::max(0.0f, distanceToNext - (stopDistance - 2.0f));
                            }
                        }
                    }

                    // Simple car-following model to avoid overlap and unrealistic tailgating.
                    const glm::vec2 right{-direction.y, direction.x};
                    const glm::vec2 selfLaneCenter = car.position + (right * laneOffset);
                    float nearestLeadDistance = std::numeric_limits<float>::max();
                    for (std::size_t j = 0; j < g_carRuntime.cars.size(); ++j)
                    {
                        if (j == i)
                        {
                            continue;
                        }

                        const agents::CarAgent &other = g_carRuntime.cars[j];
                        const glm::vec2 otherForward{std::cos(other.angle), std::sin(other.angle)};
                        const float directionAlignment = glm::dot(direction, otherForward);
                        if (directionAlignment < 0.72f)
                        {
                            continue;
                        }

                        const glm::vec2 otherRight{-otherForward.y, otherForward.x};
                        const glm::vec2 otherLaneCenter = other.position + (otherRight * laneOffset);
                        const glm::vec2 relative = otherLaneCenter - selfLaneCenter;

                        const float forwardDistance = glm::dot(relative, direction);
                        if (forwardDistance <= 0.0f)
                        {
                            continue;
                        }

                        const float lateralDistance = std::fabs(glm::dot(relative, right));
                        if (lateralDistance > 3.7f)
                        {
                            continue;
                        }

                        if (forwardDistance < nearestLeadDistance)
                        {
                            nearestLeadDistance = forwardDistance;
                        }
                    }

                    if (nearestLeadDistance < std::numeric_limits<float>::max())
                    {
                        const float minGap = 10.5f;
                        const float cautionGap = 26.0f;
                        if (nearestLeadDistance <= minGap)
                        {
                            speedScale = 0.0f;
                        }
                        else if (nearestLeadDistance < cautionGap)
                        {
                            const float followFactor = (nearestLeadDistance - minGap) / (cautionGap - minGap);
                            speedScale = std::min(speedScale, followFactor);
                        }
                    }

                    float step = car.speed * speedScale * deltaSeconds;
                    step = std::min(step, maxTravel);

                    if (step >= distanceToNext)
                    {
                        car.position = car.target;
                    }
                    else if (step > 0.0f)
                    {
                        car.position.x += step * std::cos(car.angle);
                        car.position.y += step * std::sin(car.angle);
                    }
                }
            }

            void initializeNpcs(const world::CityMap &map, int tilePixels)
            {
                if (g_npcRuntime.initialized)
                {
                    return;
                }

                std::vector<glm::vec2> buildingSpots;
                std::vector<glm::vec2> foodSpots;

                for (std::size_t row = 0; row < map.rows(); ++row)
                {
                    for (std::size_t col = 0; col < map.cols(); ++col)
                    {
                        const world::TileType tile = map.tileAt(row, col);
                        const glm::vec2 center = tileCenter(
                            static_cast<int>(row),
                            static_cast<int>(col),
                            tilePixels);

                        if (tile == world::TileType::Building)
                        {
                            buildingSpots.push_back(center);
                        }
                        else if (tile == world::TileType::Park || tile == world::TileType::Empty)
                        {
                            foodSpots.push_back(center);
                        }
                    }
                }

                if (buildingSpots.empty())
                {
                    g_npcRuntime.initialized = true;
                    return;
                }

                if (foodSpots.empty())
                {
                    foodSpots = buildingSpots;
                }

                const int npcCount = std::min(36, static_cast<int>(buildingSpots.size()));
                std::uniform_int_distribution<std::size_t> homeDist(0, buildingSpots.size() - 1U);
                std::uniform_int_distribution<std::size_t> foodDist(0, foodSpots.size() - 1U);

                for (int i = 0; i < npcCount; ++i)
                {
                    const glm::vec2 home = buildingSpots[homeDist(g_npcRuntime.rng)];

                    glm::vec2 work = buildingSpots[homeDist(g_npcRuntime.rng)];
                    int guard = 0;
                    while (work == home && guard < 12)
                    {
                        work = buildingSpots[homeDist(g_npcRuntime.rng)];
                        ++guard;
                    }

                    glm::vec2 food = foodSpots[foodDist(g_npcRuntime.rng)];
                    guard = 0;
                    while ((food == home || food == work) && guard < 12)
                    {
                        food = foodSpots[foodDist(g_npcRuntime.rng)];
                        ++guard;
                    }

                    agents::NpcAgent npc = agents::makeDefaultNpc(i, home);
                    npc.home = home;
                    npc.work = work;
                    npc.food = food;
                    npc.target = work;
                    npc.money = 80 + ((i * 17) % 140);
                    npc.state = agents::NpcState::Sleeping;
                    npc.cycleStage = 0;
                    npc.dwellSeconds = 3.0f + static_cast<float>(i % 4);
                    g_npcRuntime.npcs.push_back(npc);
                }

                g_npcRuntime.initialized = true;
            }

            void updateNpcs(float deltaSeconds)
            {
                if (deltaSeconds <= 0.0f)
                {
                    return;
                }

                const float walkSpeed = 10.0f;
                const float travelSpeed = 22.0f;

                for (agents::NpcAgent &npc : g_npcRuntime.npcs)
                {
                    if (npc.dwellSeconds > 0.0f)
                    {
                        npc.dwellSeconds -= deltaSeconds;
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
                            npc.state = agents::NpcState::Working;
                            npc.money += 20;
                            npc.dwellSeconds = 10.0f;
                            npc.target = npc.food;
                            npc.cycleStage = 1;
                        }
                        else if (npc.cycleStage == 1)
                        {
                            npc.state = agents::NpcState::Walking;
                            npc.money = std::max(0, npc.money - 8);
                            npc.dwellSeconds = 6.0f;
                            npc.target = npc.home;
                            npc.cycleStage = 2;
                        }
                        else
                        {
                            npc.state = agents::NpcState::Sleeping;
                            npc.dwellSeconds = 12.0f;
                            npc.target = npc.work;
                            npc.cycleStage = 0;
                        }

                        continue;
                    }

                    const glm::vec2 direction = toTarget / distance;
                    const float speed = (distance < 12.0f) ? walkSpeed : travelSpeed;
                    const float step = std::min(speed * deltaSeconds, distance);
                    npc.position += direction * step;
                    npc.state = (speed == walkSpeed) ? agents::NpcState::Walking : agents::NpcState::Traveling;
                }
            }

            void drawNpcs()
            {
                for (const agents::NpcAgent &npc : g_npcRuntime.npcs)
                {
                    float r = 0.70f;
                    float g = 0.74f;
                    float b = 0.79f;

                    if (npc.state == agents::NpcState::Sleeping)
                    {
                        r = 0.43f;
                        g = 0.47f;
                        b = 0.66f;
                    }
                    else if (npc.state == agents::NpcState::Working)
                    {
                        r = 0.88f;
                        g = 0.64f;
                        b = 0.26f;
                    }
                    else if (npc.state == agents::NpcState::Walking)
                    {
                        r = 0.39f;
                        g = 0.78f;
                        b = 0.41f;
                    }
                    else if (npc.state == agents::NpcState::Traveling)
                    {
                        r = 0.89f;
                        g = 0.35f;
                        b = 0.29f;
                    }

                    const int px = static_cast<int>(npc.position.x);
                    const int py = static_cast<int>(npc.position.y);

                    // Feet shadow.
                    glColor3f(0.08f, 0.09f, 0.10f);
                    drawPoints(graphics::buildFilledRectPoints(px - 2, py + 4, 4, 1));

                    // Body.
                    glColor3f(r, g, b);
                    drawPoints(graphics::buildFilledRectPoints(px - 2, py - 1, 4, 5));

                    // Head.
                    glColor3f(0.93f, 0.79f, 0.66f);
                    const std::vector<glm::vec2> head = graphics::buildCircleFanVertices(
                        {static_cast<float>(px), static_cast<float>(py - 3)},
                        2.0f,
                        12);
                    glBegin(GL_TRIANGLE_FAN);
                    for (const glm::vec2 &vertex : head)
                    {
                        glVertex2f(vertex.x, vertex.y);
                    }
                    glEnd();
                }
            }

            void drawCars()
            {
                const auto drawOrientedQuad = [](const glm::vec2 &center, const glm::vec2 &forward, const glm::vec2 &right, float halfLength, float halfWidth)
                {
                    const glm::vec2 frontLeft = center + (forward * halfLength) - (right * halfWidth);
                    const glm::vec2 frontRight = center + (forward * halfLength) + (right * halfWidth);
                    const glm::vec2 rearRight = center - (forward * halfLength) + (right * halfWidth);
                    const glm::vec2 rearLeft = center - (forward * halfLength) - (right * halfWidth);

                    glBegin(GL_QUADS);
                    glVertex2f(frontLeft.x, frontLeft.y);
                    glVertex2f(frontRight.x, frontRight.y);
                    glVertex2f(rearRight.x, rearRight.y);
                    glVertex2f(rearLeft.x, rearLeft.y);
                    glEnd();
                };

                for (const agents::CarAgent &car : g_carRuntime.cars)
                {
                    const float cosA = std::cos(car.angle);
                    const float sinA = std::sin(car.angle);
                    const glm::vec2 forward{cosA, sinA};
                    const glm::vec2 right{-sinA, cosA};

                    const float halfLength = 6.2f;
                    const float halfWidth = 3.1f;

                    // Keep cars in directional lanes: right-hand side of travel direction.
                    const float laneOffset = 2.2f;
                    const glm::vec2 center = car.position + (right * laneOffset);
                    const glm::vec2 noseCenter = center + (forward * (halfLength - 0.55f));

                    // Body color variation by id for visual differentiation.
                    const float hue = static_cast<float>((car.id * 37) % 100) / 100.0f;
                    const float bodyR = 0.30f + (0.5f * hue);
                    const float bodyG = 0.22f + (0.35f * (1.0f - hue));
                    const float bodyB = 0.28f + (0.45f * hue);

                    // Ground shadow improves depth and readability while moving.
                    glColor3f(0.06f, 0.07f, 0.08f);
                    drawOrientedQuad(center + glm::vec2(0.8f, 1.0f), forward, right, halfLength + 0.4f, halfWidth + 0.35f);

                    // Main body shell.
                    glColor3f(bodyR, bodyG, bodyB);
                    drawOrientedQuad(center, forward, right, halfLength, halfWidth);

                    // Hood and trunk color accents.
                    glColor3f(bodyR * 0.86f, bodyG * 0.86f, bodyB * 0.86f);
                    drawOrientedQuad(center + (forward * 3.2f), forward, right, 1.8f, halfWidth - 0.45f);
                    glColor3f(bodyR * 0.74f, bodyG * 0.74f, bodyB * 0.74f);
                    drawOrientedQuad(center - (forward * 3.1f), forward, right, 1.5f, halfWidth - 0.45f);

                    // Passenger cabin and roof.
                    glColor3f(0.14f, 0.17f, 0.21f);
                    drawOrientedQuad(center + (forward * 0.4f), forward, right, 2.8f, halfWidth - 0.75f);

                    glColor3f(0.56f, 0.62f, 0.68f);
                    drawOrientedQuad(center + (forward * 0.7f), forward, right, 1.2f, halfWidth - 1.1f);

                    // Windshields (front/rear) and side glass strip.
                    glColor3f(0.72f, 0.86f, 0.96f);
                    drawOrientedQuad(center + (forward * 1.95f), forward, right, 0.9f, halfWidth - 1.0f);
                    glColor3f(0.61f, 0.75f, 0.87f);
                    drawOrientedQuad(center - (forward * 1.35f), forward, right, 0.75f, halfWidth - 1.05f);

                    glColor3f(0.38f, 0.47f, 0.56f);
                    drawOrientedQuad(center + (forward * 0.25f), forward, right, 0.5f, halfWidth - 0.95f);

                    // Wheels.
                    glColor3f(0.09f, 0.10f, 0.11f);
                    drawOrientedQuad(center + (forward * 2.9f) + (right * 2.65f), forward, right, 0.95f, 0.55f);
                    drawOrientedQuad(center + (forward * 2.9f) - (right * 2.65f), forward, right, 0.95f, 0.55f);
                    drawOrientedQuad(center - (forward * 2.7f) + (right * 2.65f), forward, right, 0.95f, 0.55f);
                    drawOrientedQuad(center - (forward * 2.7f) - (right * 2.65f), forward, right, 0.95f, 0.55f);

                    // Wheel rims.
                    glColor3f(0.70f, 0.72f, 0.74f);
                    drawOrientedQuad(center + (forward * 2.9f) + (right * 2.65f), forward, right, 0.40f, 0.32f);
                    drawOrientedQuad(center + (forward * 2.9f) - (right * 2.65f), forward, right, 0.40f, 0.32f);
                    drawOrientedQuad(center - (forward * 2.7f) + (right * 2.65f), forward, right, 0.40f, 0.32f);
                    drawOrientedQuad(center - (forward * 2.7f) - (right * 2.65f), forward, right, 0.40f, 0.32f);

                    // Headlights and tail lights.
                    glColor3f(0.95f, 0.94f, 0.78f);
                    drawOrientedQuad(noseCenter + (right * 1.65f), forward, right, 0.36f, 0.24f);
                    drawOrientedQuad(noseCenter - (right * 1.65f), forward, right, 0.36f, 0.24f);

                    glColor3f(0.88f, 0.20f, 0.16f);
                    drawOrientedQuad(center - (forward * (halfLength - 0.45f)) + (right * 1.65f), forward, right, 0.36f, 0.24f);
                    drawOrientedQuad(center - (forward * (halfLength - 0.45f)) - (right * 1.65f), forward, right, 0.36f, 0.24f);

                    // Front grille and rear bumper trims.
                    glColor3f(0.10f, 0.12f, 0.13f);
                    drawOrientedQuad(center + (forward * (halfLength - 0.9f)), forward, right, 0.22f, 1.35f);
                    glColor3f(0.18f, 0.19f, 0.20f);
                    drawOrientedQuad(center - (forward * (halfLength - 0.95f)), forward, right, 0.24f, 1.30f);

                    // Roof rails for premium vehicle silhouette.
                    glColor3f(0.82f, 0.84f, 0.86f);
                    drawOrientedQuad(center + (forward * 0.8f) + (right * 0.9f), forward, right, 1.35f, 0.10f);
                    drawOrientedQuad(center + (forward * 0.8f) - (right * 0.9f), forward, right, 1.35f, 0.10f);
                }
            }

            void drawPoints(const std::vector<glm::ivec2> &points)
            {
                if (points.empty())
                {
                    return;
                }

                // Merge adjacent pixels on each scanline to avoid micro cracks at zoom.
                std::vector<glm::ivec2> sortedPoints = points;
                std::sort(sortedPoints.begin(), sortedPoints.end(), [](const glm::ivec2 &a, const glm::ivec2 &b)
                          {
                              if (a.y != b.y)
                              {
                                  return a.y < b.y;
                              }
                              return a.x < b.x; });

                glBegin(GL_QUADS);
                std::size_t i = 0;
                while (i < sortedPoints.size())
                {
                    const int y = sortedPoints[i].y;
                    int spanStartX = sortedPoints[i].x;
                    int spanEndX = sortedPoints[i].x;
                    ++i;

                    while (i < sortedPoints.size() && sortedPoints[i].y == y)
                    {
                        const int x = sortedPoints[i].x;
                        if (x <= spanEndX + 1)
                        {
                            if (x > spanEndX)
                            {
                                spanEndX = x;
                            }
                        }
                        else
                        {
                            const float left = static_cast<float>(spanStartX);
                            const float right = static_cast<float>(spanEndX + 1);
                            const float top = static_cast<float>(y);
                            const float bottom = static_cast<float>(y + 1);
                            glVertex2f(left, top);
                            glVertex2f(right, top);
                            glVertex2f(right, bottom);
                            glVertex2f(left, bottom);

                            spanStartX = x;
                            spanEndX = x;
                        }

                        ++i;
                    }

                    const float left = static_cast<float>(spanStartX);
                    const float right = static_cast<float>(spanEndX + 1);
                    const float top = static_cast<float>(y);
                    const float bottom = static_cast<float>(y + 1);
                    glVertex2f(left, top);
                    glVertex2f(right, top);
                    glVertex2f(right, bottom);
                    glVertex2f(left, bottom);
                }
                glEnd();
            }

            void drawSolidLine(int x0, int y0, int x1, int y1)
            {
                drawPoints(graphics::buildLinePointsBresenham(x0, y0, x1, y1));
            }

            void drawSolidLineSupercover(int x0, int y0, int x1, int y1)
            {
                const std::vector<glm::ivec2> basePoints = graphics::buildLinePointsBresenham(x0, y0, x1, y1);
                if (basePoints.empty())
                {
                    return;
                }

                std::vector<glm::ivec2> coverPoints;
                coverPoints.reserve(basePoints.size() * 2);
                coverPoints.push_back(basePoints.front());

                for (std::size_t i = 1; i < basePoints.size(); ++i)
                {
                    const glm::ivec2 prev = basePoints[i - 1];
                    const glm::ivec2 curr = basePoints[i];

                    const int dx = curr.x - prev.x;
                    const int dy = curr.y - prev.y;

                    if (dx != 0 && dy != 0)
                    {
                        // Add one connector pixel on diagonal transitions to avoid dotted edges.
                        coverPoints.push_back({curr.x, prev.y});
                    }

                    coverPoints.push_back(curr);
                }

                drawPoints(coverPoints);
            }

            void drawFilledTriangleBresenham(int x0, int y0, int x1, int y1, int x2, int y2)
            {
                const int minY = std::min(y0, std::min(y1, y2));
                const int maxY = std::max(y0, std::max(y1, y2));

                if (minY > maxY)
                {
                    return;
                }

                const int spanCount = (maxY - minY) + 1;
                std::vector<int> minXs(static_cast<std::size_t>(spanCount), std::numeric_limits<int>::max());
                std::vector<int> maxXs(static_cast<std::size_t>(spanCount), std::numeric_limits<int>::min());

                const auto stampEdge = [&](const std::vector<glm::ivec2> &edge)
                {
                    for (const glm::ivec2 &point : edge)
                    {
                        const int idx = point.y - minY;
                        if (idx < 0 || idx >= spanCount)
                        {
                            continue;
                        }

                        const std::size_t row = static_cast<std::size_t>(idx);
                        if (point.x < minXs[row])
                        {
                            minXs[row] = point.x;
                        }
                        if (point.x > maxXs[row])
                        {
                            maxXs[row] = point.x;
                        }
                    }
                };

                stampEdge(graphics::buildLinePointsBresenham(x0, y0, x1, y1));
                stampEdge(graphics::buildLinePointsBresenham(x1, y1, x2, y2));
                stampEdge(graphics::buildLinePointsBresenham(x2, y2, x0, y0));

                for (int y = minY; y <= maxY; ++y)
                {
                    const std::size_t row = static_cast<std::size_t>(y - minY);
                    if (minXs[row] <= maxXs[row])
                    {
                        drawSolidLine(minXs[row], y, maxXs[row], y);
                    }
                }
            }

            void drawQuad(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3)
            {
                // Fill as two triangles so walls/roofs stay solid at integer pixel resolution.
                drawFilledTriangleBresenham(x0, y0, x1, y1, x2, y2);
                drawFilledTriangleBresenham(x0, y0, x2, y2, x3, y3);
            }

            void drawTriangle(int x0, int y0, int x1, int y1, int x2, int y2)
            {
                drawFilledTriangleBresenham(x0, y0, x1, y1, x2, y2);
            }

            void drawDashedLine(int x0, int y0, int x1, int y1, int dashLength, int gapLength)
            {
                if (dashLength <= 0)
                {
                    return;
                }

                // Supports horizontal and vertical lines for lane markings.
                if (y0 == y1)
                {
                    const int start = (x0 < x1) ? x0 : x1;
                    const int end = (x0 < x1) ? x1 : x0;
                    for (int s = start; s <= end; s += (dashLength + gapLength))
                    {
                        int e = s + dashLength - 1;
                        if (e > end)
                        {
                            e = end;
                        }
                        drawSolidLine(s, y0, e, y0);
                    }
                    return;
                }

                if (x0 == x1)
                {
                    const int start = (y0 < y1) ? y0 : y1;
                    const int end = (y0 < y1) ? y1 : y0;
                    for (int s = start; s <= end; s += (dashLength + gapLength))
                    {
                        int e = s + dashLength - 1;
                        if (e > end)
                        {
                            e = end;
                        }
                        drawSolidLine(x0, s, x0, e);
                    }
                }
            }

            void drawEmpty(float x, float y, float size)
            {
                const int ix = static_cast<int>(x);
                const int iy = static_cast<int>(y);
                const int isize = static_cast<int>(size);

                // Deterministic variation so empty blocks become realistic urban spaces.
                const int variant = ((ix / isize) * 5 + (iy / isize) * 3) % 3;

                if (variant == 0)
                {
                    // Parking lot.
                    glColor3f(0.26f, 0.27f, 0.29f);
                    drawPoints(graphics::buildFilledRectPoints(ix, iy, isize, isize));

                    // Subtle perimeter shadow adds depth between blocks.
                    glColor3f(0.16f, 0.17f, 0.19f);
                    drawPoints(graphics::buildFilledRectPoints(ix, iy + isize - 3, isize, 3));
                    drawPoints(graphics::buildFilledRectPoints(ix + isize - 3, iy, 3, isize));

                    glColor3f(0.80f, 0.82f, 0.84f);
                    drawSolidLine(ix + 3, iy + 3, ix + isize - 4, iy + 3);
                    drawSolidLine(ix + 3, iy + isize - 4, ix + isize - 4, iy + isize - 4);

                    // Parking bays.
                    glColor3f(0.90f, 0.90f, 0.88f);
                    for (int px = ix + 5; px <= ix + isize - 6; px += 5)
                    {
                        drawSolidLine(px, iy + 5, px, iy + isize - 6);
                    }

                    // Parking booth and a light pole.
                    glColor3f(0.62f, 0.64f, 0.66f);
                    drawPoints(graphics::buildFilledRectPoints(ix + 2, iy + 2, 4, 3));
                    glColor3f(0.20f, 0.22f, 0.24f);
                    drawPoints(graphics::buildFilledRectPoints(ix + isize - 5, iy + 4, 1, 7));
                    glColor3f(0.96f, 0.92f, 0.68f);
                    drawPoints(graphics::buildFilledRectPoints(ix + isize - 6, iy + 3, 3, 2));
                }
                else if (variant == 1)
                {
                    // Small urban plaza.
                    glColor3f(0.52f, 0.53f, 0.52f);
                    drawPoints(graphics::buildFilledRectPoints(ix, iy, isize, isize));

                    // Center paving zone.
                    glColor3f(0.66f, 0.66f, 0.64f);
                    drawPoints(graphics::buildFilledRectPoints(ix + 4, iy + 4, isize - 8, isize - 8));

                    // Cross tiles.
                    glColor3f(0.58f, 0.58f, 0.56f);
                    drawPoints(graphics::buildFilledRectPoints(ix + (isize / 2) - 1, iy + 4, 2, isize - 8));
                    drawPoints(graphics::buildFilledRectPoints(ix + 4, iy + (isize / 2) - 1, isize - 8, 2));

                    // Decorative center circle using midpoint circle algorithm output.
                    glColor3f(0.40f, 0.42f, 0.43f);
                    const std::vector<glm::vec2> centerCircle = graphics::buildCircleFanVertices(
                        {static_cast<float>(ix + (isize / 2)), static_cast<float>(iy + (isize / 2))}, 3.0f, 16);
                    glBegin(GL_TRIANGLE_FAN);
                    for (const glm::vec2 &vertex : centerCircle)
                    {
                        glVertex2f(vertex.x, vertex.y);
                    }
                    glEnd();

                    // Seating corners and a tiny statue base.
                    glColor3f(0.46f, 0.47f, 0.47f);
                    drawPoints(graphics::buildFilledRectPoints(ix + 2, iy + 2, 3, 1));
                    drawPoints(graphics::buildFilledRectPoints(ix + isize - 5, iy + 2, 3, 1));
                    drawPoints(graphics::buildFilledRectPoints(ix + 2, iy + isize - 3, 3, 1));
                    drawPoints(graphics::buildFilledRectPoints(ix + isize - 5, iy + isize - 3, 3, 1));
                    glColor3f(0.70f, 0.72f, 0.74f);
                    drawPoints(graphics::buildFilledRectPoints(ix + (isize / 2) - 1, iy + (isize / 2) - 1, 2, 2));
                }
                else
                {
                    // Pocket green lot with trees and footpath.
                    glColor3f(0.21f, 0.49f, 0.22f);
                    drawPoints(graphics::buildFilledRectPoints(ix, iy, isize, isize));

                    glColor3f(0.30f, 0.62f, 0.31f);
                    drawPoints(graphics::buildFilledRectPoints(ix + 2, iy + 2, isize - 4, isize - 4));

                    // Footpath.
                    glColor3f(0.77f, 0.74f, 0.66f);
                    drawPoints(graphics::buildFilledRectPoints(ix + 3, iy + (isize / 2) - 1, isize - 6, 2));

                    // Tree trunks.
                    glColor3f(0.37f, 0.24f, 0.16f);
                    drawPoints(graphics::buildFilledRectPoints(ix + 6, iy + 14, 2, 4));
                    drawPoints(graphics::buildFilledRectPoints(ix + isize - 8, iy + 9, 2, 4));

                    // Tree canopies.
                    glColor3f(0.14f, 0.40f, 0.15f);
                    const std::vector<glm::vec2> canopyA = graphics::buildCircleFanVertices(
                        {static_cast<float>(ix + 7), static_cast<float>(iy + 13)}, 3.0f, 16);
                    glBegin(GL_TRIANGLE_FAN);
                    for (const glm::vec2 &vertex : canopyA)
                    {
                        glVertex2f(vertex.x, vertex.y);
                    }
                    glEnd();

                    glColor3f(0.16f, 0.45f, 0.18f);
                    const std::vector<glm::vec2> canopyB = graphics::buildCircleFanVertices(
                        {static_cast<float>(ix + isize - 7), static_cast<float>(iy + 8)}, 3.0f, 16);
                    glBegin(GL_TRIANGLE_FAN);
                    for (const glm::vec2 &vertex : canopyB)
                    {
                        glVertex2f(vertex.x, vertex.y);
                    }
                    glEnd();

                    // Picnic table + flower strip.
                    glColor3f(0.54f, 0.37f, 0.24f);
                    drawPoints(graphics::buildFilledRectPoints(ix + 4, iy + isize - 6, 5, 2));
                    glColor3f(0.86f, 0.42f, 0.58f);
                    drawPoints(graphics::buildFilledRectPoints(ix + isize - 10, iy + 3, 5, 2));
                }
            }

            void drawRoad(const world::CityMap &map, int row, int col, float x, float y, float size, float simulationSeconds)
            {
                const int ix = static_cast<int>(x);
                const int iy = static_cast<int>(y);
                const int isize = static_cast<int>(size);

                // Base asphalt.
                glColor3f(0.20f, 0.21f, 0.22f);
                drawPoints(graphics::buildFilledRectPoints(
                    ix,
                    iy,
                    isize,
                    isize));

                // Determine road direction from neighboring road tiles.
                const bool hasLeft = isRoadTile(map, row, col - 1);
                const bool hasRight = isRoadTile(map, row, col + 1);
                const bool hasUp = isRoadTile(map, row - 1, col);
                const bool hasDown = isRoadTile(map, row + 1, col);

                const bool horizontalRoad = hasLeft || hasRight;
                const bool verticalRoad = hasUp || hasDown;

                const int laneInset = 5;
                const int dashLength = 6;
                const int gapLength = 4;

                if (horizontalRoad || (!horizontalRoad && !verticalRoad))
                {
                    const int dividerY = iy + (isize / 2);

                    // White side lane boundaries: only draw outer borders, not between touching road tiles.
                    glColor3f(0.92f, 0.92f, 0.92f);
                    if (!hasUp)
                    {
                        drawSolidLine(ix, iy + laneInset, ix + isize - 1, iy + laneInset);
                    }
                    if (!hasDown)
                    {
                        drawSolidLine(ix, iy + isize - laneInset, ix + isize - 1, iy + isize - laneInset);
                    }

                    // Two-way double yellow dashed center divider.
                    glColor3f(0.94f, 0.84f, 0.20f);
                    drawDashedLine(ix, dividerY - 1, ix + isize - 1, dividerY - 1, dashLength, gapLength);
                    drawDashedLine(ix, dividerY + 1, ix + isize - 1, dividerY + 1, dashLength, gapLength);
                }

                if (verticalRoad)
                {
                    const int dividerX = ix + (isize / 2);

                    // White side lane boundaries: only draw outer borders, not between touching road tiles.
                    glColor3f(0.92f, 0.92f, 0.92f);
                    if (!hasLeft)
                    {
                        drawSolidLine(ix + laneInset, iy, ix + laneInset, iy + isize - 1);
                    }
                    if (!hasRight)
                    {
                        drawSolidLine(ix + isize - laneInset, iy, ix + isize - laneInset, iy + isize - 1);
                    }

                    // Two-way double yellow dashed center divider.
                    glColor3f(0.94f, 0.84f, 0.20f);
                    drawDashedLine(dividerX - 1, iy, dividerX - 1, iy + isize - 1, dashLength, gapLength);
                    drawDashedLine(dividerX + 1, iy, dividerX + 1, iy + isize - 1, dashLength, gapLength);
                }

                if (horizontalRoad && verticalRoad)
                {
                    // Small intersection box to visually blend crossing lane markings.
                    glColor3f(0.28f, 0.29f, 0.30f);
                    drawPoints(graphics::buildFilledRectPoints(
                        ix + (isize / 2) - 4,
                        iy + (isize / 2) - 4,
                        8,
                        8));

                    const int roadConnections =
                        (hasLeft ? 1 : 0) +
                        (hasRight ? 1 : 0) +
                        (hasUp ? 1 : 0) +
                        (hasDown ? 1 : 0);

                    const auto drawLamp = [&](int cx, int cy, systems::TrafficLightColor color, bool isActive)
                    {
                        float red = 0.15f;
                        float green = 0.15f;
                        float blue = 0.15f;

                        if (isActive)
                        {
                            if (color == systems::TrafficLightColor::Red)
                            {
                                red = 0.95f;
                                green = 0.18f;
                                blue = 0.16f;
                            }
                            else if (color == systems::TrafficLightColor::Yellow)
                            {
                                red = 0.96f;
                                green = 0.82f;
                                blue = 0.22f;
                            }
                            else
                            {
                                red = 0.20f;
                                green = 0.92f;
                                blue = 0.26f;
                            }
                        }

                        glColor3f(red, green, blue);
                        drawPoints(graphics::buildFilledRectPoints(cx - 1, cy - 1, 2, 2));
                    };

                    const auto drawSignalHead = [&](int x0, int y0, systems::TrafficLightColor activeColor)
                    {
                        glColor3f(0.12f, 0.12f, 0.13f);
                        drawPoints(graphics::buildFilledRectPoints(x0, y0, 4, 8));

                        drawLamp(x0 + 2, y0 + 2, systems::TrafficLightColor::Red, activeColor == systems::TrafficLightColor::Red);
                        drawLamp(x0 + 2, y0 + 4, systems::TrafficLightColor::Yellow, activeColor == systems::TrafficLightColor::Yellow);
                        drawLamp(x0 + 2, y0 + 6, systems::TrafficLightColor::Green, activeColor == systems::TrafficLightColor::Green);
                    };

                    if (roadConnections >= 4)
                    {
                        const systems::IntersectionSignalState signal =
                            systems::queryIntersectionSignal(
                                map,
                                row,
                                col,
                                simulationSeconds);

                        // Horizontal approach heads (east/west traffic).
                        drawSignalHead(ix + 2, iy + (isize / 2) - 4, signal.horizontal);
                        drawSignalHead(ix + isize - 6, iy + (isize / 2) - 4, signal.horizontal);

                        // Vertical approach heads (north/south traffic).
                        drawSignalHead(ix + (isize / 2) - 2, iy + 2, signal.vertical);
                        drawSignalHead(ix + (isize / 2) - 2, iy + isize - 10, signal.vertical);
                    }
                    else if (roadConnections == 3)
                    {
                        // For T-junctions, keep all active heads green to avoid broken signal behavior.
                        if (hasLeft)
                        {
                            drawSignalHead(ix + 2, iy + (isize / 2) - 4, systems::TrafficLightColor::Green);
                        }
                        if (hasRight)
                        {
                            drawSignalHead(ix + isize - 6, iy + (isize / 2) - 4, systems::TrafficLightColor::Green);
                        }
                        if (hasUp)
                        {
                            drawSignalHead(ix + (isize / 2) - 2, iy + 2, systems::TrafficLightColor::Green);
                        }
                        if (hasDown)
                        {
                            drawSignalHead(ix + (isize / 2) - 2, iy + isize - 10, systems::TrafficLightColor::Green);
                        }
                    }
                }
            }

            void drawBuilding(const world::CityMap &map, int row, int col, float x, float y, float size)
            {
                const int ix = static_cast<int>(x);
                const int iy = static_cast<int>(y);
                const int isize = static_cast<int>(size);

                const BuildingStyle style = chooseBuildingStyle(map, row, col);

                if (style == BuildingStyle::Office)
                {
                    // Ground pad around the office to anchor the perspective building.
                    glColor3f(0.24f, 0.26f, 0.28f);
                    drawPoints(graphics::buildFilledRectPoints(ix, iy, isize, isize));

                    // Plaza paving border.
                    glColor3f(0.34f, 0.36f, 0.38f);
                    drawPoints(graphics::buildFilledRectPoints(ix + 1, iy + 1, isize - 2, 2));
                    drawPoints(graphics::buildFilledRectPoints(ix + 1, iy + isize - 3, isize - 2, 2));

                    glColor3f(0.30f, 0.33f, 0.36f);
                    drawPoints(graphics::buildFilledRectPoints(ix + 3, iy + isize - 8, isize - 6, 5));

                    const int cornerX = ix + (isize / 2);
                    const int roofTopY = iy + 4;
                    const int roofEdgeY = iy + 9;
                    const int wallBottomY = iy + isize - 4;

                    // Cast shadow to give depth.
                    glColor3f(0.10f, 0.11f, 0.12f);
                    drawQuad(cornerX, wallBottomY, cornerX + 8, wallBottomY - 3, cornerX + 8, wallBottomY + 2, cornerX, wallBottomY + 4);

                    // Left and right walls from a corner viewpoint.
                    glColor3f(0.40f, 0.46f, 0.52f);
                    drawQuad(cornerX - 8, roofEdgeY, cornerX, roofEdgeY + 4, cornerX, wallBottomY, cornerX - 8, wallBottomY - 3);

                    glColor3f(0.27f, 0.31f, 0.37f);
                    drawQuad(cornerX, roofEdgeY + 4, cornerX + 8, roofEdgeY, cornerX + 8, wallBottomY - 3, cornerX, wallBottomY);

                    // Flat roof visible from above.
                    glColor3f(0.56f, 0.62f, 0.68f);
                    drawQuad(cornerX - 8, roofEdgeY, cornerX, roofTopY, cornerX + 8, roofEdgeY, cornerX, roofEdgeY + 4);

                    // Roof trim lines improve edge readability.
                    glColor3f(0.18f, 0.21f, 0.25f);
                    drawSolidLineSupercover(cornerX - 8, roofEdgeY, cornerX, roofTopY);
                    drawSolidLineSupercover(cornerX, roofTopY, cornerX + 8, roofEdgeY);
                    drawSolidLineSupercover(cornerX - 8, roofEdgeY, cornerX, roofEdgeY + 4);
                    drawSolidLineSupercover(cornerX, roofEdgeY + 4, cornerX + 8, roofEdgeY);

                    // Reflective window modules on both visible facades.
                    glColor3f(0.67f, 0.84f, 0.93f);
                    for (int yy = roofEdgeY + 3; yy <= wallBottomY - 5; yy += 4)
                    {
                        drawPoints(graphics::buildFilledRectPoints(cornerX - 6, yy, 2, 2));
                        drawPoints(graphics::buildFilledRectPoints(cornerX - 3, yy + 1, 2, 2));

                        drawPoints(graphics::buildFilledRectPoints(cornerX + 2, yy + 1, 2, 2));
                        drawPoints(graphics::buildFilledRectPoints(cornerX + 5, yy, 2, 2));
                    }

                    // Ground floor entrance and side walk.
                    glColor3f(0.12f, 0.17f, 0.23f);
                    drawPoints(graphics::buildFilledRectPoints(cornerX - 1, wallBottomY - 5, 2, 5));

                    glColor3f(0.70f, 0.72f, 0.74f);
                    drawPoints(graphics::buildFilledRectPoints(cornerX - 1, wallBottomY, 3, isize - (wallBottomY - iy)));

                    // Small rooftop HVAC unit for realism.
                    glColor3f(0.48f, 0.52f, 0.56f);
                    drawQuad(cornerX + 2, roofEdgeY - 1, cornerX + 4, roofEdgeY - 2, cornerX + 5, roofEdgeY, cornerX + 3, roofEdgeY + 1);

                    // Street furniture near office entrance.
                    glColor3f(0.22f, 0.24f, 0.26f);
                    drawPoints(graphics::buildFilledRectPoints(ix + 2, iy + isize - 4, 2, 3));
                    drawPoints(graphics::buildFilledRectPoints(ix + isize - 4, iy + isize - 4, 2, 3));
                    glColor3f(0.18f, 0.42f, 0.21f);
                    drawPoints(graphics::buildFilledRectPoints(ix + 2, iy + isize - 6, 2, 2));
                    drawPoints(graphics::buildFilledRectPoints(ix + isize - 4, iy + isize - 6, 2, 2));
                    return;
                }

                // Residential lot base.
                glColor3f(0.25f, 0.35f, 0.24f);
                drawPoints(graphics::buildFilledRectPoints(ix, iy, isize, isize));

                // Inner lawn.
                glColor3f(0.32f, 0.48f, 0.31f);
                drawPoints(graphics::buildFilledRectPoints(ix + 2, iy + 2, isize - 4, isize - 4));

                // Boundary fence around lot.
                glColor3f(0.69f, 0.63f, 0.54f);
                drawSolidLine(ix + 1, iy + 1, ix + isize - 2, iy + 1);
                drawSolidLine(ix + 1, iy + isize - 2, ix + isize - 2, iy + isize - 2);
                drawSolidLine(ix + 1, iy + 1, ix + 1, iy + isize - 2);
                drawSolidLine(ix + isize - 2, iy + 1, ix + isize - 2, iy + isize - 2);

                // Smaller corner-view house.
                const int cornerX = ix + (isize / 2) + 1;
                const int roofApexY = iy + 8;
                const int roofEdgeY = iy + 12;
                const int wallTopY = iy + 14;
                const int wallBottomY = iy + isize - 6;

                glColor3f(0.88f, 0.80f, 0.70f);
                drawQuad(cornerX - 5, wallTopY, cornerX, wallTopY + 2, cornerX, wallBottomY, cornerX - 5, wallBottomY - 2);

                glColor3f(0.78f, 0.70f, 0.60f);
                drawQuad(cornerX, wallTopY + 2, cornerX + 5, wallTopY, cornerX + 5, wallBottomY - 2, cornerX, wallBottomY);

                // Roof planes.
                glColor3f(0.62f, 0.24f, 0.20f);
                drawTriangle(cornerX - 6, roofEdgeY, cornerX, roofApexY, cornerX, wallTopY + 1);

                glColor3f(0.47f, 0.17f, 0.14f);
                drawTriangle(cornerX, roofApexY, cornerX + 6, roofEdgeY, cornerX, wallTopY + 1);

                // Roof/wall edge definition.
                glColor3f(0.32f, 0.12f, 0.10f);
                drawSolidLineSupercover(cornerX - 6, roofEdgeY, cornerX, roofApexY);
                drawSolidLineSupercover(cornerX, roofApexY, cornerX + 6, roofEdgeY);
                drawSolidLineSupercover(cornerX - 6, roofEdgeY, cornerX, wallTopY + 1);
                drawSolidLineSupercover(cornerX, wallTopY + 1, cornerX + 6, roofEdgeY);

                // Door and windows.
                glColor3f(0.35f, 0.24f, 0.17f);
                drawPoints(graphics::buildFilledRectPoints(cornerX - 1, wallBottomY - 4, 2, 4));

                glColor3f(0.74f, 0.88f, 0.95f);
                drawPoints(graphics::buildFilledRectPoints(cornerX - 4, wallTopY + 3, 2, 2));
                drawPoints(graphics::buildFilledRectPoints(cornerX + 2, wallTopY + 4, 2, 2));

                // Walkway to lot gate.
                glColor3f(0.69f, 0.68f, 0.64f);
                drawPoints(graphics::buildFilledRectPoints(cornerX, wallBottomY, 2, isize - (wallBottomY - iy) - 2));

                // Small backyard pool.
                glColor3f(0.21f, 0.59f, 0.84f);
                drawPoints(graphics::buildFilledRectPoints(ix + 3, iy + 4, 6, 4));
                glColor3f(0.54f, 0.78f, 0.92f);
                drawPoints(graphics::buildFilledRectPoints(ix + 4, iy + 5, 4, 2));

                // Swing set.
                const int swingBaseY = iy + isize - 5;
                glColor3f(0.52f, 0.40f, 0.32f);
                drawSolidLine(ix + 4, swingBaseY, ix + 6, swingBaseY - 4);
                drawSolidLine(ix + 10, swingBaseY, ix + 8, swingBaseY - 4);
                drawSolidLine(ix + 6, swingBaseY - 4, ix + 8, swingBaseY - 4);

                glColor3f(0.24f, 0.24f, 0.24f);
                drawSolidLine(ix + 6, swingBaseY - 4, ix + 6, swingBaseY - 2);
                drawSolidLine(ix + 8, swingBaseY - 4, ix + 8, swingBaseY - 2);
                drawPoints(graphics::buildFilledRectPoints(ix + 6, swingBaseY - 2, 3, 1));

                // Two decorative trees.
                glColor3f(0.39f, 0.25f, 0.17f);
                drawPoints(graphics::buildFilledRectPoints(ix + isize - 7, iy + 5, 2, 4));
                drawPoints(graphics::buildFilledRectPoints(ix + isize - 6, iy + isize - 9, 2, 4));

                glColor3f(0.15f, 0.43f, 0.18f);
                const std::vector<glm::vec2> treeA = graphics::buildCircleFanVertices(
                    {static_cast<float>(ix + isize - 6), static_cast<float>(iy + 5)}, 3.0f, 16);
                glBegin(GL_TRIANGLE_FAN);
                for (const glm::vec2 &vertex : treeA)
                {
                    glVertex2f(vertex.x, vertex.y);
                }
                glEnd();

                glColor3f(0.17f, 0.47f, 0.20f);
                const std::vector<glm::vec2> treeB = graphics::buildCircleFanVertices(
                    {static_cast<float>(ix + isize - 5), static_cast<float>(iy + isize - 9)}, 3.0f, 16);
                glBegin(GL_TRIANGLE_FAN);
                for (const glm::vec2 &vertex : treeB)
                {
                    glVertex2f(vertex.x, vertex.y);
                }
                glEnd();

                // Mailbox and driveway edge.
                glColor3f(0.12f, 0.24f, 0.60f);
                drawPoints(graphics::buildFilledRectPoints(ix + 2, iy + isize - 4, 2, 3));
                glColor3f(0.62f, 0.62f, 0.60f);
                drawPoints(graphics::buildFilledRectPoints(ix + 9, iy + isize - 3, 6, 2));
            }

            void drawPark(float x, float y, float size)
            {
                const int ix = static_cast<int>(x);
                const int iy = static_cast<int>(y);
                const int isize = static_cast<int>(size);

                // Base grass.
                glColor3f(0.22f, 0.58f, 0.26f);
                drawPoints(graphics::buildFilledRectPoints(ix, iy, isize, isize));

                // Border hedge ring.
                glColor3f(0.16f, 0.46f, 0.20f);
                drawPoints(graphics::buildFilledRectPoints(ix + 1, iy + 1, isize - 2, 1));
                drawPoints(graphics::buildFilledRectPoints(ix + 1, iy + isize - 2, isize - 2, 1));
                drawPoints(graphics::buildFilledRectPoints(ix + 1, iy + 1, 1, isize - 2));
                drawPoints(graphics::buildFilledRectPoints(ix + isize - 2, iy + 1, 1, isize - 2));

                // Inner lawn patch for depth.
                glColor3f(0.30f, 0.66f, 0.32f);
                drawPoints(graphics::buildFilledRectPoints(ix + 3, iy + 3, isize - 6, isize - 6));

                // Curved-ish walking path made from two connected strips.
                glColor3f(0.78f, 0.74f, 0.63f);
                drawPoints(graphics::buildFilledRectPoints(ix + 2, iy + (isize / 2) - 2, isize - 4, 4));
                drawPoints(graphics::buildFilledRectPoints(ix + (isize / 2) - 2, iy + 2, 4, isize - 4));

                // Small pond.
                glColor3f(0.28f, 0.62f, 0.86f);
                drawPoints(graphics::buildFilledRectPoints(ix + isize - 11, iy + 4, 6, 4));
                drawPoints(graphics::buildFilledRectPoints(ix + isize - 10, iy + 8, 4, 2));
                glColor3f(0.66f, 0.88f, 0.96f);
                drawPoints(graphics::buildFilledRectPoints(ix + isize - 10, iy + 5, 3, 1));

                // Tree canopies (circles from midpoint algorithm).
                glColor3f(0.14f, 0.44f, 0.19f);
                const std::vector<glm::vec2> tree1 = graphics::buildCircleFanVertices(
                    {static_cast<float>(ix + 7), static_cast<float>(iy + 8)}, 3.0f, 16);
                drawPoints(graphics::buildFilledRectPoints(ix + 5, iy + 10, 2, 4));
                glBegin(GL_TRIANGLE_FAN);
                for (const glm::vec2 &vertex : tree1)
                {
                    glVertex2f(vertex.x, vertex.y);
                }
                glEnd();

                glColor3f(0.16f, 0.48f, 0.21f);
                const std::vector<glm::vec2> tree2 = graphics::buildCircleFanVertices(
                    {static_cast<float>(ix + isize - 7), static_cast<float>(iy + isize - 8)}, 3.0f, 16);
                drawPoints(graphics::buildFilledRectPoints(ix + isize - 9, iy + isize - 6, 2, 4));
                glBegin(GL_TRIANGLE_FAN);
                for (const glm::vec2 &vertex : tree2)
                {
                    glVertex2f(vertex.x, vertex.y);
                }
                glEnd();

                // Flower bed accent.
                glColor3f(0.92f, 0.38f, 0.56f);
                drawPoints(graphics::buildFilledRectPoints(ix + 4, iy + isize - 7, 5, 2));

                // Benches and lamp post.
                glColor3f(0.53f, 0.37f, 0.24f);
                drawPoints(graphics::buildFilledRectPoints(ix + 4, iy + 4, 4, 1));
                drawPoints(graphics::buildFilledRectPoints(ix + isize - 9, iy + isize - 5, 4, 1));
                glColor3f(0.24f, 0.24f, 0.25f);
                drawPoints(graphics::buildFilledRectPoints(ix + (isize / 2), iy + 5, 1, 8));
                glColor3f(0.95f, 0.93f, 0.70f);
                drawPoints(graphics::buildFilledRectPoints(ix + (isize / 2) - 1, iy + 4, 3, 2));
            }

        } // namespace

        void Renderer::render(int viewportWidth, int viewportHeight) const
        {
            static world::CityMap cityMap;
            static bool initialized = false;
            if (!initialized)
            {
                cityMap.initializeDefault();
                initialized = true;
            }

            // Setup a simple tile-space camera so each map cell is a 1x1 block.
            const std::size_t cols = cityMap.cols();
            const std::size_t rows = cityMap.rows();
            const int tilePixels = 28;
            const int mapWidthPixels = static_cast<int>(cols) * tilePixels;
            const int mapHeightPixels = static_cast<int>(rows) * tilePixels;

            const int safeViewportWidth = (viewportWidth <= 0) ? 1 : viewportWidth;
            const int safeViewportHeight = (viewportHeight <= 0) ? 1 : viewportHeight;
            const float simulationSeconds = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;

            initializeCars(cityMap, tilePixels);
            float carDeltaSeconds = simulationSeconds - g_carRuntime.lastSimulationSeconds;
            if (g_carRuntime.lastSimulationSeconds <= 0.0f)
            {
                carDeltaSeconds = 0.0f;
            }
            carDeltaSeconds = std::clamp(carDeltaSeconds, 0.0f, 0.05f);
            updateCars(carDeltaSeconds, cityMap, tilePixels, simulationSeconds);
            g_carRuntime.lastSimulationSeconds = simulationSeconds;

            initializeNpcs(cityMap, tilePixels);
            float npcDeltaSeconds = simulationSeconds - g_npcRuntime.lastSimulationSeconds;
            if (g_npcRuntime.lastSimulationSeconds <= 0.0f)
            {
                npcDeltaSeconds = 0.0f;
            }
            npcDeltaSeconds = std::clamp(npcDeltaSeconds, 0.0f, 0.05f);
            updateNpcs(npcDeltaSeconds);
            g_npcRuntime.lastSimulationSeconds = simulationSeconds;

            updateNavigation(safeViewportWidth, safeViewportHeight, mapWidthPixels, mapHeightPixels);
            const CameraState camera = cameraState();
            const float safeZoom = (camera.zoom <= 0.01f) ? 0.01f : camera.zoom;

            const float viewHalfHeight = (static_cast<float>(mapHeightPixels) * 0.5f) / safeZoom;
            const float viewHalfWidth = viewHalfHeight *
                                        (static_cast<float>(safeViewportWidth) / static_cast<float>(safeViewportHeight));

            // Fill the entire window to avoid letterboxing/pillarboxing.
            glViewport(0, 0, safeViewportWidth, safeViewportHeight);
            glDisable(GL_BLEND);
            glDisable(GL_POINT_SMOOTH);
            glDisable(GL_LINE_SMOOTH);
            glDisable(GL_POLYGON_SMOOTH);
            glClear(GL_COLOR_BUFFER_BIT);

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(
                static_cast<double>(camera.centerX - viewHalfWidth),
                static_cast<double>(camera.centerX + viewHalfWidth),
                static_cast<double>(camera.centerY + viewHalfHeight),
                static_cast<double>(camera.centerY - viewHalfHeight),
                -1.0,
                1.0);

            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

            // Soft atmospheric base tint under the whole city block.
            glBegin(GL_QUADS);
            glColor3f(0.11f, 0.15f, 0.18f);
            glVertex2f(0.0f, 0.0f);
            glColor3f(0.13f, 0.17f, 0.20f);
            glVertex2f(static_cast<float>(mapWidthPixels), 0.0f);
            glColor3f(0.18f, 0.20f, 0.18f);
            glVertex2f(static_cast<float>(mapWidthPixels), static_cast<float>(mapHeightPixels));
            glColor3f(0.16f, 0.19f, 0.17f);
            glVertex2f(0.0f, static_cast<float>(mapHeightPixels));
            glEnd();

            // Render tile map by looping through each cell and drawing by type.
            for (std::size_t row = 0; row < cityMap.rows(); ++row)
            {
                for (std::size_t col = 0; col < cityMap.cols(); ++col)
                {
                    const int rowIndex = static_cast<int>(row);
                    const int colIndex = static_cast<int>(col);
                    const float x = static_cast<float>(static_cast<int>(col) * tilePixels);
                    const float y = static_cast<float>(static_cast<int>(row) * tilePixels);
                    const float tileSize = static_cast<float>(tilePixels);

                    switch (cityMap.tileAt(row, col))
                    {
                    case world::TileType::Road:
                        drawRoad(cityMap, rowIndex, colIndex, x, y, tileSize, simulationSeconds);
                        break;
                    case world::TileType::Building:
                        drawBuilding(cityMap, rowIndex, colIndex, x, y, tileSize);
                        break;
                    case world::TileType::Park:
                        drawPark(x, y, tileSize);
                        break;
                    default:
                        drawEmpty(x, y, tileSize);
                        break;
                    }
                }
            }

            drawNpcs();
            drawCars();
        }

    } // namespace engine

} // namespace civitasx
