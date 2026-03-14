#include "engine/renderer.h"

#include <GL/freeglut.h>

#include "world/city_map.h"

namespace civitasx
{

    namespace engine
    {

        namespace
        {

            void drawQuad(float x0, float y0, float x1, float y1)
            {
                glBegin(GL_QUADS);
                glVertex2f(x0, y0);
                glVertex2f(x1, y0);
                glVertex2f(x1, y1);
                glVertex2f(x0, y1);
                glEnd();
            }

            void drawEmpty(float x, float y, float size)
            {
                glColor3f(0.08f, 0.08f, 0.08f);
                drawQuad(x, y, x + size, y + size);
            }

            void drawRoad(float x, float y, float size)
            {
                glColor3f(0.35f, 0.35f, 0.35f);
                drawQuad(x, y, x + size, y + size);
            }

            void drawBuilding(float x, float y, float size)
            {
                glColor3f(0.70f, 0.55f, 0.25f);
                drawQuad(x, y, x + size, y + size);
            }

            void drawPark(float x, float y, float size)
            {
                glColor3f(0.20f, 0.60f, 0.25f);
                drawQuad(x, y, x + size, y + size);
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
            const float cols = static_cast<float>(cityMap.cols());
            const float rows = static_cast<float>(cityMap.rows());

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(0.0, cols, rows, 0.0, -1.0, 1.0);

            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

            // Render tile map by looping through each cell and drawing by type.
            for (std::size_t row = 0; row < cityMap.rows(); ++row)
            {
                for (std::size_t col = 0; col < cityMap.cols(); ++col)
                {
                    const float x = static_cast<float>(col);
                    const float y = static_cast<float>(row);
                    const float tileSize = 1.0f;

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
            glLineWidth(2.0f);
            glBegin(GL_LINES);
            for (std::size_t row = 0; row <= cityMap.rows(); ++row)
            {
                const float y = static_cast<float>(row);
                glVertex2f(0.0f, y);
                glVertex2f(cols, y);
            }
            for (std::size_t col = 0; col <= cityMap.cols(); ++col)
            {
                const float x = static_cast<float>(col);
                glVertex2f(x, 0.0f);
                glVertex2f(x, rows);
            }
            glEnd();
        }

    } // namespace engine

} // namespace civitasx
