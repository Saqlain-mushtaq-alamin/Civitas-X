#include "engine/renderer.h"

#include "engine/camera.h"
#include "graphics/shape_drawer.h"
#include "graphics/transformation.h"
#include "systems/simulation_manager.h"

#include <GL/freeglut.h>

namespace civitasx
{

    namespace engine
    {

        void Renderer::render(const systems::SimulationManager &simulation, int viewportWidth, int viewportHeight) const
        {
            glClear(GL_COLOR_BUFFER_BIT);

            const world::CityMapConfig &cityMap = simulation.cityMap();

            Camera camera;
            camera.applyOrtho(viewportWidth, viewportHeight, cityMap);

            glColor3f(0.10f, 0.13f, 0.16f);
            glBegin(GL_QUADS);
            glVertex2f(-cityMap.halfWidth, -cityMap.halfHeight);
            glVertex2f(cityMap.halfWidth, -cityMap.halfHeight);
            glVertex2f(cityMap.halfWidth, cityMap.halfHeight);
            glVertex2f(-cityMap.halfWidth, cityMap.halfHeight);
            glEnd();

            glColor3f(0.20f, 0.23f, 0.27f);
            glLineWidth(8.0f);
            glBegin(GL_LINES);
            for (const glm::vec2 &node : simulation.waypoints())
            {
                glVertex2f(node.x, -cityMap.halfHeight);
                glVertex2f(node.x, cityMap.halfHeight);
                glVertex2f(-cityMap.halfWidth, node.y);
                glVertex2f(cityMap.halfWidth, node.y);
            }
            glEnd();

            glColor3f(0.15f, 0.75f, 0.90f);
            for (const auto &car : simulation.cars())
            {
                const glm::vec2 position = graphics::clampToCityBounds(car.position, cityMap);
                graphics::drawFilledCircle(position, 2.8f, 18);
            }

            glColor3f(0.98f, 0.95f, 0.60f);
            glPointSize(4.0f);
            glBegin(GL_POINTS);
            for (const glm::vec2 &node : simulation.waypoints())
            {
                const glm::vec2 waypoint = graphics::clampToCityBounds(node, cityMap);
                glVertex2f(waypoint.x, waypoint.y);
            }
            glEnd();
        }

    } // namespace engine

} // namespace civitasx
