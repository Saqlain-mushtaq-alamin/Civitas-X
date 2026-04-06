#pragma once

#include <glm/vec2.hpp>

namespace civitasx
{

    namespace agents
    {

        enum class CarState : int
        {
            Free = 0,
            Assigned = 1,
            GoToPickup = 2,
            WaitForNpc = 3,
            Transporting = 4,
        };

        struct CarAgent
        {
            int id = 0;
            glm::vec2 position{0.0f, 0.0f};
            glm::vec2 target{0.0f, 0.0f};
            glm::vec2 pickupLocation{0.0f, 0.0f};
            glm::vec2 destination{0.0f, 0.0f};
            float speed = 25.0f;
            float currentSpeed = 0.0f;
            float angle = 0.0f;
            float battery = 100.0f;
            float wallet = 50.0f;
            CarState state = CarState::Free;
            int passengerNpcId = -1;

            // Rental/fueling logic
            bool isRented = false;
            int renterNpcId = -1;
            bool isFueling = false;
        };

        // Update function for simulation
        void updateCarAgent(CarAgent &car, float deltaSeconds, float distanceTraveled);

        CarAgent makeDefaultCar(int id, const glm::vec2 &spawnPosition, float speed);

    } // namespace agents

} // namespace civitasx
