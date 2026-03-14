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

            void drawEmpty(float x, float y, float size)
            {
                glColor3f(0.08f, 0.08f, 0.08f);
                drawPoints(graphics::buildFilledRectPoints(
                    static_cast<int>(x),
                    static_cast<int>(y),
                    static_cast<int>(size),
                    static_cast<int>(size)));
            }

            void drawRoad(float x, float y, float size)
            {
                glColor3f(0.35f, 0.35f, 0.35f);
                drawPoints(graphics::buildFilledRectPoints(
                    static_cast<int>(x),
                    static_cast<int>(y),
                    static_cast<int>(size),
                    static_cast<int>(size)));
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
                    const float x = static_cast<float>(static_cast<int>(col) * tilePixels);
                    const float y = static_cast<float>(static_cast<int>(row) * tilePixels);
                    const float tileSize = static_cast<float>(tilePixels);

                    switch (cityMap.tileAt(row, col))
                    {
                    case world::TileType::Road:
                        drawRoad(x, y, tileSize);
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

            // Draw grid lines to make tile boundaries clearly visible.
            glColor3f(0.0f, 0.0f, 0.0f);
            for (std::size_t row = 0; row <= rows; ++row)
            {
                const int y = static_cast<int>(row) * tilePixels;
                drawPoints(graphics::buildLinePointsBresenham(0, y, mapWidthPixels - 1, y));
            }
            for (std::size_t col = 0; col <= cols; ++col)
            {
                const int x = static_cast<int>(col) * tilePixels;
                drawPoints(graphics::buildLinePointsBresenham(x, 0, x, mapHeightPixels - 1));
            }
        }

    } // namespace engine

} // namespace civitasx
