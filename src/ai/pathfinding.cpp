#include "ai/pathfinding.h"

#include <algorithm>
#include <limits>

#include <glm/geometric.hpp>

namespace civitasx
{

    namespace ai
    {

        namespace
        {
            int tileIndexFor(std::size_t row, std::size_t col, std::size_t cols)
            {
                return static_cast<int>((row * cols) + col);
            }

            float nodeDistance(const RoadGraph &graph, int a, int b)
            {
                return glm::length(
                    graph.nodeCenters[static_cast<std::size_t>(a)] -
                    graph.nodeCenters[static_cast<std::size_t>(b)]);
            }
        } // namespace

        glm::vec2 chooseNextWaypoint(
            std::mt19937 &rng,
            const std::vector<glm::vec2> &waypoints,
            const glm::vec2 &currentPosition)
        {
            if (waypoints.empty())
            {
                return currentPosition;
            }

            std::uniform_int_distribution<std::size_t> pointDist(0, waypoints.size() - 1U);
            glm::vec2 next = waypoints[pointDist(rng)];

            int guard = 0;
            while (next == currentPosition && guard < 8)
            {
                next = waypoints[pointDist(rng)];
                ++guard;
            }

            return next;
        }

        RoadGraph buildRoadGraph(const world::CityMap &map, int tilePixels)
        {
            RoadGraph graph;
            graph.rows = map.rows();
            graph.cols = map.cols();
            graph.nodeByTile.assign(graph.rows * graph.cols, -1);

            for (std::size_t row = 0; row < graph.rows; ++row)
            {
                for (std::size_t col = 0; col < graph.cols; ++col)
                {
                    if (map.tileAt(row, col) != world::TileType::Road)
                    {
                        continue;
                    }

                    const int nodeId = static_cast<int>(graph.nodeCenters.size());
                    const int tileIndex = tileIndexFor(row, col, graph.cols);
                    graph.nodeByTile[static_cast<std::size_t>(tileIndex)] = nodeId;
                    graph.nodeCenters.push_back(
                        {static_cast<float>(static_cast<int>(col) * tilePixels + (tilePixels / 2)),
                         static_cast<float>(static_cast<int>(row) * tilePixels + (tilePixels / 2))});
                    graph.nodeRowsById.push_back(static_cast<int>(row));
                    graph.nodeColsById.push_back(static_cast<int>(col));
                }
            }

            graph.adjacency.assign(graph.nodeCenters.size(), {});
            const int dRows[4] = {-1, 1, 0, 0};
            const int dCols[4] = {0, 0, -1, 1};

            for (std::size_t row = 0; row < graph.rows; ++row)
            {
                for (std::size_t col = 0; col < graph.cols; ++col)
                {
                    const int fromTile = tileIndexFor(row, col, graph.cols);
                    const int fromNode = graph.nodeByTile[static_cast<std::size_t>(fromTile)];
                    if (fromNode < 0)
                    {
                        continue;
                    }

                    for (int i = 0; i < 4; ++i)
                    {
                        const int nRow = static_cast<int>(row) + dRows[i];
                        const int nCol = static_cast<int>(col) + dCols[i];
                        if (nRow < 0 || nCol < 0)
                        {
                            continue;
                        }

                        const std::size_t neighborRow = static_cast<std::size_t>(nRow);
                        const std::size_t neighborCol = static_cast<std::size_t>(nCol);
                        if (neighborRow >= graph.rows || neighborCol >= graph.cols)
                        {
                            continue;
                        }

                        const int toTile = tileIndexFor(neighborRow, neighborCol, graph.cols);
                        const int toNode = graph.nodeByTile[static_cast<std::size_t>(toTile)];
                        if (toNode >= 0)
                        {
                            graph.adjacency[static_cast<std::size_t>(fromNode)].push_back(toNode);
                        }
                    }
                }
            }

            return graph;
        }

        bool chooseRandomGoalNode(std::mt19937 &rng, const RoadGraph &graph, int startNode, int &goalNode)
        {
            if (graph.nodeCenters.size() < 2U)
            {
                return false;
            }

            std::uniform_int_distribution<std::size_t> nodeDist(0, graph.nodeCenters.size() - 1U);
            for (int attempts = 0; attempts < 16; ++attempts)
            {
                const int candidate = static_cast<int>(nodeDist(rng));
                if (candidate != startNode)
                {
                    goalNode = candidate;
                    return true;
                }
            }

            return false;
        }

        std::vector<int> findPathAStar(const RoadGraph &graph, int startNode, int goalNode)
        {
            std::vector<int> empty;
            const int nodeCount = static_cast<int>(graph.nodeCenters.size());
            if (startNode < 0 || startNode >= nodeCount || goalNode < 0 || goalNode >= nodeCount)
            {
                return empty;
            }

            if (startNode == goalNode)
            {
                return {startNode};
            }

            const float inf = std::numeric_limits<float>::infinity();
            std::vector<float> gScore(static_cast<std::size_t>(nodeCount), inf);
            std::vector<float> fScore(static_cast<std::size_t>(nodeCount), inf);
            std::vector<int> cameFrom(static_cast<std::size_t>(nodeCount), -1);
            std::vector<bool> inOpen(static_cast<std::size_t>(nodeCount), false);
            std::vector<int> openSet;
            openSet.reserve(static_cast<std::size_t>(nodeCount));

            gScore[static_cast<std::size_t>(startNode)] = 0.0f;
            fScore[static_cast<std::size_t>(startNode)] = nodeDistance(graph, startNode, goalNode);
            openSet.push_back(startNode);
            inOpen[static_cast<std::size_t>(startNode)] = true;

            while (!openSet.empty())
            {
                std::size_t bestIndex = 0;
                float bestScore = fScore[static_cast<std::size_t>(openSet[0])];
                for (std::size_t i = 1; i < openSet.size(); ++i)
                {
                    const float score = fScore[static_cast<std::size_t>(openSet[i])];
                    if (score < bestScore)
                    {
                        bestScore = score;
                        bestIndex = i;
                    }
                }

                const int current = openSet[bestIndex];
                openSet.erase(openSet.begin() + static_cast<std::ptrdiff_t>(bestIndex));
                inOpen[static_cast<std::size_t>(current)] = false;

                if (current == goalNode)
                {
                    std::vector<int> path;
                    int trace = goalNode;
                    while (trace >= 0)
                    {
                        path.push_back(trace);
                        trace = cameFrom[static_cast<std::size_t>(trace)];
                    }
                    std::reverse(path.begin(), path.end());
                    return path;
                }

                for (int neighbor : graph.adjacency[static_cast<std::size_t>(current)])
                {
                    const float tentative =
                        gScore[static_cast<std::size_t>(current)] + nodeDistance(graph, current, neighbor);

                    if (tentative >= gScore[static_cast<std::size_t>(neighbor)])
                    {
                        continue;
                    }

                    cameFrom[static_cast<std::size_t>(neighbor)] = current;
                    gScore[static_cast<std::size_t>(neighbor)] = tentative;
                    fScore[static_cast<std::size_t>(neighbor)] = tentative + nodeDistance(graph, neighbor, goalNode);

                    if (!inOpen[static_cast<std::size_t>(neighbor)])
                    {
                        openSet.push_back(neighbor);
                        inOpen[static_cast<std::size_t>(neighbor)] = true;
                    }
                }
            }

            return empty;
        }

    } // namespace ai

} // namespace civitasx
