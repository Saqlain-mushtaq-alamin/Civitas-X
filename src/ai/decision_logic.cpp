#include "ai/decision_logic.h"

namespace civitasx
{

    namespace ai
    {

        void refreshDestinationIfReached(agents::CarAgent &car, const std::vector<glm::vec2> &waypoints)
        {
            if (waypoints.empty())
            {
                return;
            }

            const float epsilon = 0.001f;
            const glm::vec2 delta = car.target - car.position;
            if ((delta.x * delta.x + delta.y * delta.y) < epsilon)
            {
                car.target = waypoints.front();
            }
        }

    } // namespace ai

} // namespace civitasx
