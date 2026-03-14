#include "graphics/shape_drawer.h"

#include "graphics/algorithms.h"

#include <GL/freeglut.h>

namespace civitasx
{

    namespace graphics
    {

        void drawFilledCircle(const glm::vec2 &center, float radius, int segments)
        {
            const std::vector<glm::vec2> vertices = buildCircleFanVertices(center, radius, segments);
            if (vertices.empty())
            {
                return;
            }

            glBegin(GL_TRIANGLE_FAN);
            for (const glm::vec2 &vertex : vertices)
            {
                glVertex2f(vertex.x, vertex.y);
            }
            glEnd();
        }

    } // namespace graphics

} // namespace civitasx
