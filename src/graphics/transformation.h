#pragma once

#include <glm/vec2.hpp>

#include "world/city_map.h"

namespace civitasx
{

    namespace graphics
    {

        glm::vec2 clampToCityBounds(const glm::vec2 &position, const world::CityMapConfig &cityMap);

    } // namespace graphics

} // namespace civitasx
