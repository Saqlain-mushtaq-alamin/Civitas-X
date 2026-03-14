#include "ai/behavior_system.h"

#include <algorithm>

namespace civitasx
{

    namespace ai
    {

        float desiredSpeedForCar(const agents::CarAgent &car)
        {
            const float batteryFactor = std::clamp(car.battery / 100.0f, 0.35f, 1.0f);
            return car.speed * batteryFactor;
        }

    } // namespace ai

} // namespace civitasx
