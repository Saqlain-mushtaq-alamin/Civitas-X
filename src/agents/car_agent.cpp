#include "agents/car_agent.h"

namespace civitasx
{

    namespace agents
    {

        CarAgent makeDefaultCar(int id, const glm::vec2 &spawnPosition, float speed)
        {
            CarAgent car;
            car.id = id;
            car.position = spawnPosition;
            car.target = spawnPosition;
            car.speed = speed;
            car.battery = 100.0f;
            car.wallet = 50.0f;
            return car;
        }

    } // namespace agents

} // namespace civitasx
