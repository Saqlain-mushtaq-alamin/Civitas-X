#include "graphics/algorithms.h"

#include <cmath>

namespace civitasx
{

    namespace graphics
    {

        std::vector<glm::vec2> buildCircleFanVertices(const glm::vec2 &center, float radius, int segments)
        {
            std::vector<glm::vec2> vertices;
            if (segments < 3 || radius <= 0.0f)
            {
                return vertices;
            }

            vertices.reserve(static_cast<std::size_t>(segments) + 2U);
            vertices.push_back(center);

            for (int i = 0; i <= segments; ++i)
            {
                const float angle = static_cast<float>(i) / static_cast<float>(segments) * 6.2831853f;
                vertices.push_back({
                    center.x + radius * std::cos(angle),
                    center.y + radius * std::sin(angle),
                });
            }

            return vertices;
        }

    } // namespace graphics

} // namespace civitasx
