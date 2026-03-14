#include "world/city_map.h"

namespace civitasx
{

    namespace world
    {

        void CityMap::initializeDefault()
        {
            config_ = CityMapConfig{};

            // 20x20 tile layout. A larger active grid makes each tile look smaller on screen.
            // 0 = empty, 1 = road, 3 = park, 4 = home, 5 = office.
            for (auto &row : map_)
            {
                row.fill(static_cast<int>(TileType::Empty));
            }

            activeRows_ = 20;
            activeCols_ = 20;

            const int layout[10][10] = {
                {4, 1, 4, 0, 4, 1, 3, 3, 5, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {3, 1, 4, 4, 5, 1, 3, 3, 5, 1},
                {4, 1, 4, 0, 5, 1, 4, 3, 5, 1},
                {4, 1, 4, 5, 5, 1, 4, 5, 5, 1},
                {4, 1, 2, 3, 2, 1, 2, 3, 2, 1},
                {1, 1, 4, 3, 5, 1, 4, 3, 5, 1},
                {4, 1, 0, 3, 0, 1, 4, 0, 5, 1},
                {2, 1, 2, 0, 2, 1, 2, 2, 2, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            };

            for (std::size_t row = 0; row < activeRows_; ++row)
            {
                for (std::size_t col = 0; col < activeCols_; ++col)
                {
                    map_[row][col] = layout[row % 10][col % 10];
                }
            }
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
            case 4:
            case 5:
                return TileType::Building;
            case 3:
                return TileType::Park;
            default:
                return TileType::Empty;
            }
        }

        int CityMap::rawTileAt(std::size_t row, std::size_t col) const
        {
            if (row >= activeRows_ || col >= activeCols_)
            {
                return static_cast<int>(TileType::Empty);
            }

            return map_[row][col];
        }

    }

}
