#pragma once

#include <glm/vec2.hpp>

namespace civitasx
{

    namespace world
    {

        class TileSystem
        {
        public:
            glm::vec2 snapToGrid(const glm::vec2 &position, float tileSize) const;
        };

    } // namespace world

} // namespace civitasx
