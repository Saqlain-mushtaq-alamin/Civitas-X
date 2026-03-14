#include "systems/traffic_system.h"

#include <algorithm>
#include <cmath>

#include <glm/geometric.hpp>

namespace civitasx
{

    namespace systems
    {

        namespace
        {
            bool isRoad(const world::CityMap &map, int row, int col)
            {
                if (row < 0 || col < 0)
                {
                    return false;
                }

                const std::size_t rowIndex = static_cast<std::size_t>(row);
                const std::size_t colIndex = static_cast<std::size_t>(col);
                if (rowIndex >= map.rows() || colIndex >= map.cols())
                {
                    return false;
                }

                return map.tileAt(rowIndex, colIndex) == world::TileType::Road;
            }

            int countRoadRun(const world::CityMap &map, int row, int col, int dRow, int dCol, int maxSteps)
            {
                int runLength = 0;
                for (int step = 1; step <= maxSteps; ++step)
                {
                    const int currentRow = row + (dRow * step);
                    const int currentCol = col + (dCol * step);
                    if (!isRoad(map, currentRow, currentCol))
                    {
                        break;
                    }
                    ++runLength;
                }
                return runLength;
            }

            float clampf(float value, float minValue, float maxValue)
            {
                return std::max(minValue, std::min(value, maxValue));
            }
        } // namespace

        float advanceCar(agents::CarAgent &car, float deltaSeconds)
        {
            if (deltaSeconds <= 0.0f)
            {
                return 0.0f;
            }

            const glm::vec2 toTarget = car.target - car.position;
            const float distance = glm::length(toTarget);
            if (distance < 0.001f)
            {
                return 0.0f;
            }

            const float step = car.speed * deltaSeconds;
            const glm::vec2 direction = toTarget / distance;

            if (step >= distance)
            {
                car.position = car.target;
                return distance;
            }

            car.position += direction * step;
            return step;
        }

        bool isSignalizedIntersection(const world::CityMap &map, int row, int col)
        {
            if (!isRoad(map, row, col))
            {
                return false;
            }

            const bool hasLeft = isRoad(map, row, col - 1);
            const bool hasRight = isRoad(map, row, col + 1);
            const bool hasUp = isRoad(map, row - 1, col);
            const bool hasDown = isRoad(map, row + 1, col);
            return (hasLeft && hasRight && hasUp && hasDown);
        }

        IntersectionSignalState queryIntersectionSignal(
            const world::CityMap &map,
            int row,
            int col,
            float simulationSeconds)
        {
            IntersectionSignalState state;
            if (!isSignalizedIntersection(map, row, col))
            {
                return state;
            }

            // Measure corridor strength in each axis to bias green splits.
            const float verticalDemand = static_cast<float>(
                countRoadRun(map, row, col, -1, 0, 6) + countRoadRun(map, row, col, 1, 0, 6));
            const float horizontalDemand = static_cast<float>(
                countRoadRun(map, row, col, 0, -1, 6) + countRoadRun(map, row, col, 0, 1, 6));

            const float totalDemand = std::max(1.0f, verticalDemand + horizontalDemand);
            const float verticalShare = clampf(verticalDemand / totalDemand, 0.35f, 0.65f);

            // Fixed yellow/all-red for safety, adaptive green split for realism.
            const float yellowDuration = 2.0f;
            const float allRedDuration = 1.0f;
            const float totalGreenBudget = 14.0f;
            const float verticalGreen = clampf(totalGreenBudget * verticalShare, 5.0f, 9.0f);
            const float horizontalGreen = totalGreenBudget - verticalGreen;

            const float cycleSeconds =
                horizontalGreen + yellowDuration + allRedDuration +
                verticalGreen + yellowDuration + allRedDuration;

            // Offset nearby intersections to create progression rather than full sync.
            const float offset = std::fmod(
                (static_cast<float>(col) * 0.65f) + (static_cast<float>(row) * 0.45f),
                cycleSeconds);

            float cycleTime = std::fmod(simulationSeconds + offset, cycleSeconds);
            if (cycleTime < 0.0f)
            {
                cycleTime += cycleSeconds;
            }

            if (cycleTime < horizontalGreen)
            {
                state.horizontal = TrafficLightColor::Green;
                state.vertical = TrafficLightColor::Red;
                return state;
            }

            cycleTime -= horizontalGreen;
            if (cycleTime < yellowDuration)
            {
                state.horizontal = TrafficLightColor::Yellow;
                state.vertical = TrafficLightColor::Red;
                return state;
            }

            cycleTime -= yellowDuration;
            if (cycleTime < allRedDuration)
            {
                state.horizontal = TrafficLightColor::Red;
                state.vertical = TrafficLightColor::Red;
                return state;
            }

            cycleTime -= allRedDuration;
            if (cycleTime < verticalGreen)
            {
                state.horizontal = TrafficLightColor::Red;
                state.vertical = TrafficLightColor::Green;
                return state;
            }

            cycleTime -= verticalGreen;
            if (cycleTime < yellowDuration)
            {
                state.horizontal = TrafficLightColor::Red;
                state.vertical = TrafficLightColor::Yellow;
                return state;
            }

            state.horizontal = TrafficLightColor::Red;
            state.vertical = TrafficLightColor::Red;
            return state;
        }

    } // namespace systems

} // namespace civitasx
