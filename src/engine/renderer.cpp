#include "engine/renderer.h"

#include <GL/freeglut.h>

#include "graphics/algorithms.h"
#include "world/city_map.h"

namespace civitasx
{

    namespace engine
    {

        namespace
        {

            bool isRoadTile(const world::CityMap &map, int row, int col)
            {
                if (row < 0 || col < 0)
                {
                    return false;
                }

                const std::size_t rowIndex = static_cast<std::size_t>(row);
                const std::size_t colIndex = static_cast<std::size_t>(col);
                if (rowIndex >= map.rows() || colIndex >= map.cols())
                {
                    return false;
                }

                return map.tileAt(rowIndex, colIndex) == world::TileType::Road;
            }

            void drawPoints(const std::vector<glm::ivec2> &points)
            {
                // Keep algorithm-based point generation, but draw each point as a 1x1 cell.
                // This avoids dotted artifacts when the scene is scaled to window size.
                glBegin(GL_QUADS);
                for (const glm::ivec2 &point : points)
                {
                    const float x = static_cast<float>(point.x);
                    const float y = static_cast<float>(point.y);
                    glVertex2f(x, y);
                    glVertex2f(x + 1.0f, y);
                    glVertex2f(x + 1.0f, y + 1.0f);
                    glVertex2f(x, y + 1.0f);
                }
                glEnd();
            }

            void drawSolidLine(int x0, int y0, int x1, int y1)
            {
                drawPoints(graphics::buildLinePointsBresenham(x0, y0, x1, y1));
            }

            void drawDashedLine(int x0, int y0, int x1, int y1, int dashLength, int gapLength)
            {
                if (dashLength <= 0)
                {
                    return;
                }

                // Supports horizontal and vertical lines for lane markings.
                if (y0 == y1)
                {
                    const int start = (x0 < x1) ? x0 : x1;
                    const int end = (x0 < x1) ? x1 : x0;
                    for (int s = start; s <= end; s += (dashLength + gapLength))
                    {
                        int e = s + dashLength - 1;
                        if (e > end)
                        {
                            e = end;
                        }
                        drawSolidLine(s, y0, e, y0);
                    }
                    return;
                }

                if (x0 == x1)
                {
                    const int start = (y0 < y1) ? y0 : y1;
                    const int end = (y0 < y1) ? y1 : y0;
                    for (int s = start; s <= end; s += (dashLength + gapLength))
                    {
                        int e = s + dashLength - 1;
                        if (e > end)
                        {
                            e = end;
                        }
                        drawSolidLine(x0, s, x0, e);
                    }
                }
            }

            void drawEmpty(float x, float y, float size)
            {
                glColor3f(0.08f, 0.08f, 0.08f);
                drawPoints(graphics::buildFilledRectPoints(
                    static_cast<int>(x),
                    static_cast<int>(y),
                    static_cast<int>(size),
                    static_cast<int>(size)));
            }

            void drawRoad(const world::CityMap &map, int row, int col, float x, float y, float size)
            {
                const int ix = static_cast<int>(x);
                const int iy = static_cast<int>(y);
                const int isize = static_cast<int>(size);

                // Base asphalt.
                glColor3f(0.20f, 0.21f, 0.22f);
                drawPoints(graphics::buildFilledRectPoints(
                    ix,
                    iy,
                    isize,
                    isize));

                // Determine road direction from neighboring road tiles.
                const bool hasLeft = isRoadTile(map, row, col - 1);
                const bool hasRight = isRoadTile(map, row, col + 1);
                const bool hasUp = isRoadTile(map, row - 1, col);
                const bool hasDown = isRoadTile(map, row + 1, col);

                const bool horizontalRoad = hasLeft || hasRight;
                const bool verticalRoad = hasUp || hasDown;

                const int laneInset = 5;
                const int dashLength = 6;
                const int gapLength = 4;

                if (horizontalRoad || (!horizontalRoad && !verticalRoad))
                {
                    const int dividerY = iy + (isize / 2);

                    // White side lane boundaries: only draw outer borders, not between touching road tiles.
                    glColor3f(0.92f, 0.92f, 0.92f);
                    if (!hasUp)
                    {
                        drawSolidLine(ix, iy + laneInset, ix + isize - 1, iy + laneInset);
                    }
                    if (!hasDown)
                    {
                        drawSolidLine(ix, iy + isize - laneInset, ix + isize - 1, iy + isize - laneInset);
                    }

                    // Two-way double yellow dashed center divider.
                    glColor3f(0.94f, 0.84f, 0.20f);
                    drawDashedLine(ix, dividerY - 1, ix + isize - 1, dividerY - 1, dashLength, gapLength);
                    drawDashedLine(ix, dividerY + 1, ix + isize - 1, dividerY + 1, dashLength, gapLength);
                }

                if (verticalRoad)
                {
                    const int dividerX = ix + (isize / 2);

                    // White side lane boundaries: only draw outer borders, not between touching road tiles.
                    glColor3f(0.92f, 0.92f, 0.92f);
                    if (!hasLeft)
                    {
                        drawSolidLine(ix + laneInset, iy, ix + laneInset, iy + isize - 1);
                    }
                    if (!hasRight)
                    {
                        drawSolidLine(ix + isize - laneInset, iy, ix + isize - laneInset, iy + isize - 1);
                    }

                    // Two-way double yellow dashed center divider.
                    glColor3f(0.94f, 0.84f, 0.20f);
                    drawDashedLine(dividerX - 1, iy, dividerX - 1, iy + isize - 1, dashLength, gapLength);
                    drawDashedLine(dividerX + 1, iy, dividerX + 1, iy + isize - 1, dashLength, gapLength);
                }

                if (horizontalRoad && verticalRoad)
                {
                    // Small intersection box to visually blend crossing lane markings.
                    glColor3f(0.28f, 0.29f, 0.30f);
                    drawPoints(graphics::buildFilledRectPoints(
                        ix + (isize / 2) - 4,
                        iy + (isize / 2) - 4,
                        8,
                        8));
                }
            }

            void drawBuilding(float x, float y, float size)
            {
                glColor3f(0.70f, 0.55f, 0.25f);
                drawPoints(graphics::buildFilledRectPoints(
                    static_cast<int>(x),
                    static_cast<int>(y),
                    static_cast<int>(size),
                    static_cast<int>(size)));
            }

            void drawPark(float x, float y, float size)
            {
                glColor3f(0.20f, 0.60f, 0.25f);
                drawPoints(graphics::buildFilledRectPoints(
                    static_cast<int>(x),
                    static_cast<int>(y),
                    static_cast<int>(size),
                    static_cast<int>(size)));
            }

        } // namespace

        void Renderer::render(int viewportWidth, int viewportHeight) const
        {
            glViewport(0, 0, (viewportWidth <= 0) ? 1 : viewportWidth, (viewportHeight <= 0) ? 1 : viewportHeight);
            glClear(GL_COLOR_BUFFER_BIT);

            static world::CityMap cityMap;
            static bool initialized = false;
            if (!initialized)
            {
                cityMap.initializeDefault();
                initialized = true;
            }

            // Setup a simple tile-space camera so each map cell is a 1x1 block.
            const std::size_t cols = cityMap.cols();
            const std::size_t rows = cityMap.rows();
            const int tilePixels = 28;
            const int mapWidthPixels = static_cast<int>(cols) * tilePixels;
            const int mapHeightPixels = static_cast<int>(rows) * tilePixels;

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(0.0, mapWidthPixels, mapHeightPixels, 0.0, -1.0, 1.0);

            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

            // Render tile map by looping through each cell and drawing by type.
            for (std::size_t row = 0; row < cityMap.rows(); ++row)
            {
                for (std::size_t col = 0; col < cityMap.cols(); ++col)
                {
                    const int rowIndex = static_cast<int>(row);
                    const int colIndex = static_cast<int>(col);
                    const float x = static_cast<float>(static_cast<int>(col) * tilePixels);
                    const float y = static_cast<float>(static_cast<int>(row) * tilePixels);
                    const float tileSize = static_cast<float>(tilePixels);

                    switch (cityMap.tileAt(row, col))
                    {
                    case world::TileType::Road:
                        drawRoad(cityMap, rowIndex, colIndex, x, y, tileSize);
                        break;
                    case world::TileType::Building:
                        drawBuilding(x, y, tileSize);
                        break;
                    case world::TileType::Park:
                        drawPark(x, y, tileSize);
                        break;
                    default:
                        drawEmpty(x, y, tileSize);
                        break;
                    }
                }
            }
        }

    } // namespace engine

} // namespace civitasx
