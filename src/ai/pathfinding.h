#pragma once

#include <cstddef>
#include <random>
#include <vector>

#include <glm/vec2.hpp>

#include "world/city_map.h"

namespace civitasx
{

    namespace ai
    {

        struct RoadGraph
        {
            std::vector<glm::vec2> nodeCenters;
            std::vector<int> nodeRowsById;
            std::vector<int> nodeColsById;
            std::vector<std::vector<int>> adjacency;
            std::vector<int> nodeByTile;
            std::size_t rows = 0;
            std::size_t cols = 0;
        };

        glm::vec2 chooseNextWaypoint(
            std::mt19937 &rng,
            const std::vector<glm::vec2> &waypoints,
            const glm::vec2 &currentPosition);

        RoadGraph buildRoadGraph(const world::CityMap &map, int tilePixels);
        bool chooseRandomGoalNode(std::mt19937 &rng, const RoadGraph &graph, int startNode, int &goalNode);
        std::vector<int> findPathAStar(const RoadGraph &graph, int startNode, int goalNode);

    } // namespace ai

} // namespace civitasx
