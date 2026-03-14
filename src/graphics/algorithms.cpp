#include "graphics/algorithms.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace civitasx
{

    namespace graphics
    {

        std::vector<glm::ivec2> buildLinePointsBresenham(int x0, int y0, int x1, int y1)
        {
            std::vector<glm::ivec2> points;

            // Bresenham uses only integer math, which is ideal for raster-style lab algorithms.
            const int dx = std::abs(x1 - x0);
            const int sx = (x0 < x1) ? 1 : -1;
            const int dy = -std::abs(y1 - y0);
            const int sy = (y0 < y1) ? 1 : -1;
            int error = dx + dy;

            int x = x0;
            int y = y0;

            while (true)
            {
                points.push_back({x, y});
                if (x == x1 && y == y1)
                {
                    break;
                }

                const int twiceError = 2 * error;
                if (twiceError >= dy)
                {
                    error += dy;
                    x += sx;
                }
                if (twiceError <= dx)
                {
                    error += dx;
                    y += sy;
                }
            }

            return points;
        }

        std::vector<glm::ivec2> buildFilledRectPoints(int x, int y, std::int32_t width, std::int32_t height)
        {
            std::vector<glm::ivec2> points;
            if (width <= 0 || height <= 0)
            {
                return points;
            }

            // Fill by scanline: each row is a Bresenham horizontal line.
            for (std::int32_t row = 0; row < height; ++row)
            {
                const int yRow = y + static_cast<int>(row);
                std::vector<glm::ivec2> line =
                    buildLinePointsBresenham(x, yRow, x + static_cast<int>(width) - 1, yRow);
                points.insert(points.end(), line.begin(), line.end());
            }

            return points;
        }

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
