#pragma once

#include "agents/car_agent.h"

namespace civitasx
{

    namespace systems
    {

        void applyOperationalCosts(agents::CarAgent &car, float distanceTraveled);

    } // namespace systems

} // namespace civitasx
