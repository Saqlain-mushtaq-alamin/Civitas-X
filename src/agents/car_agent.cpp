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
            car.currentSpeed = speed;
            car.angle = 0.0f;
            car.battery = 100.0f;
            car.wallet = 50.0f;
            return car;
        }

        void updateCarAgent(CarAgent &car, float deltaSeconds, float distanceTraveled)
        {
            // If rented, consume battery and collect rent
            if (car.isRented)
            {
                car.battery -= distanceTraveled * 0.2f; // Energy cost
                car.wallet += 0.1f * deltaSeconds;      // Earn rent per second
                if (car.battery < 10.0f)
                {
                    car.isFueling = true;
                    car.isRented = false;
                    car.renterNpcId = -1;
                }
            }
            // If fueling, recharge battery
            if (car.isFueling)
            {
                car.battery += 20.0f * deltaSeconds; // Recharge rate
                if (car.battery >= 100.0f)
                {
                    car.battery = 100.0f;
                    car.isFueling = false;
                }
            }
        }

    } // namespace agents

} // namespace civitasx
