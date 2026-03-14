#include "ai/pathfinding.h"

namespace civitasx
{

    namespace ai
    {

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

    } // namespace ai

} // namespace civitasx
