#include "civitasx/renderer2d.hpp"

#include "civitasx/world.hpp"

#include <GL/freeglut.h>
#include <cmath>

namespace civitasx
{

    namespace
    {

        void drawFilledCircle(float x, float y, float radius, int segments)
        {
            glBegin(GL_TRIANGLE_FAN);
            glVertex2f(x, y);
            for (int i = 0; i <= segments; ++i)
            {
                const float angle = static_cast<float>(i) / static_cast<float>(segments) * 6.2831853f;
                glVertex2f(x + radius * std::cos(angle), y + radius * std::sin(angle));
            }
            glEnd();
        }

    } // namespace

    void Renderer2D::render(const World &world, int viewportWidth, int viewportHeight) const
    {
        glClear(GL_COLOR_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        const float aspect = static_cast<float>(viewportWidth) / static_cast<float>(viewportHeight);
        const float worldHalfHeight = 70.0f;
        const float worldHalfWidth = worldHalfHeight * aspect;
        glOrtho(-worldHalfWidth, worldHalfWidth, -worldHalfHeight, worldHalfHeight, -1.0, 1.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glColor3f(0.10f, 0.13f, 0.16f);
        glBegin(GL_QUADS);
        glVertex2f(-200.0f, -200.0f);
        glVertex2f(200.0f, -200.0f);
        glVertex2f(200.0f, 200.0f);
        glVertex2f(-200.0f, 200.0f);
        glEnd();

        glColor3f(0.20f, 0.23f, 0.27f);
        glLineWidth(8.0f);
        glBegin(GL_LINES);
        for (const auto &node : world.waypoints())
        {
            glVertex2f(node.x, -70.0f);
            glVertex2f(node.x, 70.0f);
            glVertex2f(-130.0f, node.y);
            glVertex2f(130.0f, node.y);
        }
        glEnd();

        glColor3f(0.15f, 0.75f, 0.90f);
        for (const auto &car : world.cars())
        {
            drawFilledCircle(car.position.x, car.position.y, 2.8f, 18);
        }

        glColor3f(0.98f, 0.95f, 0.60f);
        glPointSize(4.0f);
        glBegin(GL_POINTS);
        for (const auto &node : world.waypoints())
        {
            glVertex2f(node.x, node.y);
        }
        glEnd();
    }

} // namespace civitasx
