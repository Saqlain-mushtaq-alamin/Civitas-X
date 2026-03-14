#pragma once

#include <vector>

#include <glm/vec2.hpp>

namespace civitasx
{

    namespace graphics
    {

        // Builds a triangle-fan friendly vertex list for a filled circle.
        // Boundary points are produced via the Midpoint Circle algorithm.
        std::vector<glm::vec2> buildCircleFanVertices(const glm::vec2 &center, float radius, int segments);

    } // namespace graphics

} // namespace civitasx
