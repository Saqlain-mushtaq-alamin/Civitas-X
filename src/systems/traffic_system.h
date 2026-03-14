#pragma once

#include "agents/car_agent.h"
#include "world/city_map.h"

namespace civitasx
{

    namespace systems
    {

        enum class TrafficLightColor
        {
            Red,
            Yellow,
            Green,
        };

        struct IntersectionSignalState
        {
            TrafficLightColor horizontal = TrafficLightColor::Red;
            TrafficLightColor vertical = TrafficLightColor::Red;
        };

        float advanceCar(agents::CarAgent &car, float deltaSeconds);
        bool isSignalizedIntersection(const world::CityMap &map, int row, int col);
        IntersectionSignalState queryIntersectionSignal(
            const world::CityMap &map,
            int row,
            int col,
            float simulationSeconds);

    } // namespace systems

} // namespace civitasx
