#pragma once

#include <vector>

#include <glm/vec2.hpp>

#include "world/city_map.h"

namespace civitasx
{

    namespace world
    {

        class RoadNetwork
        {
        public:
            std::vector<glm::vec2> buildWaypoints(const CityMapConfig &cityMap) const;
        };

    } // namespace world

} // namespace civitasx
