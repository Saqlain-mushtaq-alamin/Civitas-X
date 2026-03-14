#pragma once

namespace civitasx
{

    namespace world
    {

        struct CityMapConfig
        {
            float halfWidth = 130.0f;
            float halfHeight = 70.0f;
            float tileSize = 5.0f;
        };

        class CityMap
        {
        public:
            void initializeDefault();
            const CityMapConfig &config() const;

        private:
            CityMapConfig config_;
        };

    } // namespace world

} // namespace civitasx
