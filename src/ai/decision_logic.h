#pragma once

#include <vector>

#include <glm/vec2.hpp>

#include "agents/car_agent.h"

namespace civitasx
{

    namespace ai
    {

        void refreshDestinationIfReached(agents::CarAgent &car, const std::vector<glm::vec2> &waypoints);

    } // namespace ai

} // namespace civitasx
