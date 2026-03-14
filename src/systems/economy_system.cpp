#include "systems/economy_system.h"

#include <algorithm>

namespace civitasx
{

    namespace systems
    {

        void applyOperationalCosts(agents::CarAgent &car, float distanceTraveled)
        {
            const float energyDrain = 0.035f * distanceTraveled;
            car.battery = std::max(0.0f, car.battery - energyDrain);

            if (car.battery <= 0.0f)
            {
                car.battery = 100.0f;
                car.wallet = std::max(0.0f, car.wallet - 3.0f);
            }
        }

    } // namespace systems

} // namespace civitasx
