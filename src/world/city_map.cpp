#include "world/city_map.h"

namespace civitasx
{

    namespace world
    {

        void CityMap::initializeDefault()
        {
            config_ = CityMapConfig{};

            // 10x10 storage with active 3x3 sample in top-left:
            // H R O
            // R R R
            // P R H
            // H/O are encoded as Building (2), R=1, P=3.
            for (auto &row : map_)
            {
                row.fill(static_cast<int>(TileType::Empty));
            }

            activeRows_ = 3;
            activeCols_ = 3;

            map_[0][0] = static_cast<int>(TileType::Building); // H
            map_[0][1] = static_cast<int>(TileType::Road);     // R
            map_[0][2] = static_cast<int>(TileType::Building); // O

            map_[1][0] = static_cast<int>(TileType::Road);
            map_[1][1] = static_cast<int>(TileType::Road);
            map_[1][2] = static_cast<int>(TileType::Road);

            map_[2][0] = static_cast<int>(TileType::Park);     // P
            map_[2][1] = static_cast<int>(TileType::Road);     // R
            map_[2][2] = static_cast<int>(TileType::Building); // H
        }

        const CityMapConfig &CityMap::config() const
        {
            return config_;
        }

        std::size_t CityMap::rows() const
        {
            return activeRows_;
        }

        std::size_t CityMap::cols() const
        {
            return activeCols_;
        }

        TileType CityMap::tileAt(std::size_t row, std::size_t col) const
        {
            if (row >= activeRows_ || col >= activeCols_)
            {
                return TileType::Empty;
            }

            const int value = map_[row][col];
            switch (value)
            {
            case 1:
                return TileType::Road;
            case 2:
                return TileType::Building;
            case 3:
                return TileType::Park;
            default:
                return TileType::Empty;
            }
        }

    }  

}  
