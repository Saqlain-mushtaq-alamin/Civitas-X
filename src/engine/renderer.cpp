#include "engine/renderer.h"

#include <GL/freeglut.h>

namespace civitasx
{

    namespace engine
    {

        void Renderer::render(int viewportWidth, int viewportHeight) const
        {
            glViewport(0, 0, (viewportWidth <= 0) ? 1 : viewportWidth, (viewportHeight <= 0) ? 1 : viewportHeight);
            glClear(GL_COLOR_BUFFER_BIT);
        }

    } // namespace engine

} // namespace civitasx
