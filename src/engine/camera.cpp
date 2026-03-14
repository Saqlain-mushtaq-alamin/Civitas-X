#include "engine/camera.h"

#include "world/city_map.h"

#include <GL/freeglut.h>

namespace civitasx
{

    namespace engine
    {

        void Camera::applyOrtho(int viewportWidth, int viewportHeight, const world::CityMapConfig &cityMap) const
        {
            const int safeHeight = (viewportHeight <= 0) ? 1 : viewportHeight;

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();

            const float aspect = static_cast<float>(viewportWidth) / static_cast<float>(safeHeight);
            const float mapHalfHeight = cityMap.halfHeight;
            const float mapHalfWidth = mapHalfHeight * aspect;
            glOrtho(-mapHalfWidth, mapHalfWidth, -mapHalfHeight, mapHalfHeight, -1.0, 1.0);

            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
        }

    } // namespace engine

} // namespace civitasx
