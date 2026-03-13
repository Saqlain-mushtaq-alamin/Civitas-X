#include "civitasx/world.hpp"

#include <algorithm>
#include <cmath>

#include <glm/geometric.hpp>

namespace civitasx
{

    void World::initialize(unsigned int seed, int carCount)
    {
        rng_.seed(seed);
        cars_.clear();
        waypoints_.clear();

        waypoints_ = {
            {-90.0f, -50.0f},
            {-45.0f, -50.0f},
            {0.0f, -50.0f},
            {45.0f, -50.0f},
            {90.0f, -50.0f},
            {-90.0f, 0.0f},
            {-45.0f, 0.0f},
            {0.0f, 0.0f},
            {45.0f, 0.0f},
            {90.0f, 0.0f},
            {-90.0f, 50.0f},
            {-45.0f, 50.0f},
            {0.0f, 50.0f},
            {45.0f, 50.0f},
            {90.0f, 50.0f},
        };

        std::uniform_int_distribution<std::size_t> pointDist(0, waypoints_.size() - 1);
        std::uniform_real_distribution<float> speedDist(18.0f, 34.0f);

        for (int i = 0; i < carCount; ++i)
        {
            CarAgent car;
            car.id = i;
            car.position = waypoints_[pointDist(rng_)];
            car.speed = speedDist(rng_);
            car.battery = 100.0f;
            car.wallet = 50.0f;
            assignNewTarget(car);
            cars_.push_back(car);
        }
    }

    void World::update(float deltaSeconds)
    {
        for (CarAgent &car : cars_)
        {
            glm::vec2 toTarget = car.target - car.position;
            const float distance = glm::length(toTarget);

            if (distance < 0.001f)
            {
                assignNewTarget(car);
                continue;
            }

            const float step = car.speed * deltaSeconds;
            glm::vec2 direction = toTarget / distance;

            if (step >= distance)
            {
                car.position = car.target;
                assignNewTarget(car);
            }
            else
            {
                car.position += direction * step;
            }

            const float energyDrain = 0.035f * step;
            car.battery = std::max(0.0f, car.battery - energyDrain);

            if (car.battery <= 0.0f)
            {
                car.battery = 100.0f;
                car.wallet = std::max(0.0f, car.wallet - 3.0f);
            }
        }
    }

    const std::vector<CarAgent> &World::cars() const
    {
        return cars_;
    }

    const std::vector<glm::vec2> &World::waypoints() const
    {
        return waypoints_;
    }

    void World::assignNewTarget(CarAgent &car)
    {
        if (waypoints_.empty())
        {
            return;
        }

        std::uniform_int_distribution<std::size_t> pointDist(0, waypoints_.size() - 1);
        glm::vec2 next = waypoints_[pointDist(rng_)];

        int guard = 0;
        while (next == car.position && guard < 8)
        {
            next = waypoints_[pointDist(rng_)];
            ++guard;
        }

        car.target = next;
    }

} // namespace civitasx
