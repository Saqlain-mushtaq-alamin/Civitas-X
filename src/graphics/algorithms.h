#pragma once

#include <vector>

#include <glm/vec2.hpp>

namespace civitasx
{

    namespace graphics
    {

        std::vector<glm::vec2> buildCircleFanVertices(const glm::vec2 &center, float radius, int segments);

    } // namespace graphics

} // namespace civitasx
