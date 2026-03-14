#include "world/road_network.h"

namespace civitasx
{

    namespace world
    {

        std::vector<glm::vec2> RoadNetwork::buildWaypoints(const CityMapConfig &cityMap) const
        {
            std::vector<glm::vec2> waypoints;

            for (float y = -cityMap.halfHeight + 20.0f; y <= cityMap.halfHeight - 20.0f; y += 50.0f)
            {
                for (float x = -cityMap.halfWidth + 40.0f; x <= cityMap.halfWidth - 40.0f; x += 45.0f)
                {
                    waypoints.push_back({x, y});
                }
            }

            return waypoints;
        }

    } // namespace world

} // namespace civitasx
