#include "world/city_map.h"

namespace civitasx
{

    namespace world
    {

        void CityMap::initializeDefault()
        {
            config_ = CityMapConfig{};
        }

        const CityMapConfig &CityMap::config() const
        {
            return config_;
        }

    } // namespace world

} // namespace civitasx
