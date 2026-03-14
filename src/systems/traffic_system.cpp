#include "systems/traffic_system.h"

#include <glm/geometric.hpp>

namespace civitasx
{

    namespace systems
    {

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

    } // namespace systems

} // namespace civitasx
