#pragma once

#include <array>
#include <cstddef>

namespace civitasx
{

    namespace world
    {

        enum class TileType : int
        {
            Empty = 0,
            Road = 1,
            Building = 2,
            Park = 3,
            Home = 4,
            Office = 5,
        };

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

            std::size_t rows() const;
            std::size_t cols() const;
            TileType tileAt(std::size_t row, std::size_t col) const;
            int rawTileAt(std::size_t row, std::size_t col) const;

        private:
            static constexpr std::size_t kMaxRows = 20;
            static constexpr std::size_t kMaxCols = 20;

            CityMapConfig config_;
            std::array<std::array<int, kMaxCols>, kMaxRows> map_{};
            std::size_t activeRows_ = 0;
            std::size_t activeCols_ = 0;
        };

    } // namespace world

} // namespace civitasx
