#pragma once

#include <random>
#include <vector>

#include <glm/vec2.hpp>

namespace civitasx
{

    namespace ai
    {

        glm::vec2 chooseNextWaypoint(
            std::mt19937 &rng,
            const std::vector<glm::vec2> &waypoints,
            const glm::vec2 &currentPosition);

    } // namespace ai

} // namespace civitasx
