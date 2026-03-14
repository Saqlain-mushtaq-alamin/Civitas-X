#include "graphics/transformation.h"

#include <algorithm>

namespace civitasx
{

    namespace graphics
    {

        glm::vec2 clampToCityBounds(const glm::vec2 &position, const world::CityMapConfig &cityMap)
        {
            return {
                std::clamp(position.x, -cityMap.halfWidth, cityMap.halfWidth),
                std::clamp(position.y, -cityMap.halfHeight, cityMap.halfHeight),
            };
        }

    } // namespace graphics

} // namespace civitasx
