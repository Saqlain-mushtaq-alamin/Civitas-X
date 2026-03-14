#pragma once

namespace civitasx
{

    namespace world
    {
        struct CityMapConfig;
    }

    namespace engine
    {

        class Camera
        {
        public:
            void applyOrtho(int viewportWidth, int viewportHeight, const world::CityMapConfig &cityMap) const;
        };

    } // namespace engine

} // namespace civitasx
