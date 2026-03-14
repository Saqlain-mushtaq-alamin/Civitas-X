#include "world/tile_system.h"

#include <cmath>

namespace civitasx
{

    namespace world
    {

        glm::vec2 TileSystem::snapToGrid(const glm::vec2 &position, float tileSize) const
        {
            if (tileSize <= 0.0f)
            {
                return position;
            }

            return {
                std::round(position.x / tileSize) * tileSize,
                std::round(position.y / tileSize) * tileSize,
            };
        }

    } // namespace world

} // namespace civitasx
