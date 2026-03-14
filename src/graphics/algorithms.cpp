#include "graphics/algorithms.h"

#include <algorithm>
#include <cmath>

namespace civitasx
{

    namespace graphics
    {

        namespace
        {

            void addUniquePoint(std::vector<glm::vec2> &points, const glm::vec2 &point)
            {
                for (const glm::vec2 &existing : points)
                {
                    if (existing.x == point.x && existing.y == point.y)
                    {
                        return;
                    }
                }
                points.push_back(point);
            }

            // Midpoint circle draws one octant and mirrors to all 8 symmetric octants.
            void plotCircleOctants(std::vector<glm::vec2> &points, const glm::vec2 &center, int x, int y)
            {
                addUniquePoint(points, {center.x + static_cast<float>(x), center.y + static_cast<float>(y)});
                addUniquePoint(points, {center.x + static_cast<float>(y), center.y + static_cast<float>(x)});
                addUniquePoint(points, {center.x - static_cast<float>(x), center.y + static_cast<float>(y)});
                addUniquePoint(points, {center.x - static_cast<float>(y), center.y + static_cast<float>(x)});
                addUniquePoint(points, {center.x + static_cast<float>(x), center.y - static_cast<float>(y)});
                addUniquePoint(points, {center.x + static_cast<float>(y), center.y - static_cast<float>(x)});
                addUniquePoint(points, {center.x - static_cast<float>(x), center.y - static_cast<float>(y)});
                addUniquePoint(points, {center.x - static_cast<float>(y), center.y - static_cast<float>(x)});
            }

        } // namespace

        std::vector<glm::vec2> buildCircleFanVertices(const glm::vec2 &center, float radius, int /*segments*/)
        {
            std::vector<glm::vec2> boundaryPoints;
            if (radius <= 0.0f)
            {
                return boundaryPoints;
            }

            const int integerRadius = std::max(1, static_cast<int>(std::lround(radius)));

            int x = 0;
            int y = integerRadius;

            // Decision value controls whether the next point moves East or South-East.
            int decision = 1 - integerRadius;

            while (x <= y)
            {
                plotCircleOctants(boundaryPoints, center, x, y);

                if (decision < 0)
                {
                    // Choose East pixel.
                    decision += (2 * x) + 3;
                }
                else
                {
                    // Choose South-East pixel and move inward on Y.
                    decision += (2 * (x - y)) + 5;
                    --y;
                }

                ++x;
            }

            // Triangle fan needs points ordered around the center.
            std::sort(boundaryPoints.begin(), boundaryPoints.end(), [&center](const glm::vec2 &a, const glm::vec2 &b)
                      {
                const float angleA = std::atan2(a.y - center.y, a.x - center.x);
                const float angleB = std::atan2(b.y - center.y, b.x - center.x);
                return angleA < angleB; });

            std::vector<glm::vec2> vertices;
            vertices.reserve(boundaryPoints.size() + 2U);
            vertices.push_back(center);
            vertices.insert(vertices.end(), boundaryPoints.begin(), boundaryPoints.end());

            // Repeat the first edge vertex so GL_TRIANGLE_FAN closes the shape cleanly.
            if (!boundaryPoints.empty())
            {
                vertices.push_back(boundaryPoints.front());
            }

            return vertices;
        }

    } // namespace graphics

} // namespace civitasx
