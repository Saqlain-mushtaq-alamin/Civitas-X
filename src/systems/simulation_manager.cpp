#include "systems/simulation_manager.h"

#include <random>

#include "ai/behavior_system.h"
#include "ai/pathfinding.h"
#include "systems/economy_system.h"
#include "systems/traffic_system.h"
#include "world/road_network.h"
#include "world/tile_system.h"

namespace civitasx
{

    namespace systems
    {

        void SimulationManager::initialize(unsigned int seed, int carCount)
        {
            rng_.seed(seed);

            cityMap_.initializeDefault();

            world::RoadNetwork roads;
            waypoints_ = roads.buildWaypoints(cityMap_.config());

            world::TileSystem tiles;
            for (glm::vec2 &point : waypoints_)
            {
                point = tiles.snapToGrid(point, cityMap_.config().tileSize);
            }

            cars_.clear();
            npcs_.clear();

            if (waypoints_.empty())
            {
                return;
            }

            std::uniform_int_distribution<std::size_t> pointDist(0, waypoints_.size() - 1U);
            std::uniform_real_distribution<float> speedDist(18.0f, 34.0f);

            for (int i = 0; i < carCount; ++i)
            {
                const glm::vec2 spawn = waypoints_[pointDist(rng_)];
                agents::CarAgent car = agents::makeDefaultCar(i, spawn, speedDist(rng_));
                car.target = ai::chooseNextWaypoint(rng_, waypoints_, car.position);
                cars_.push_back(car);
            }
        }

        void SimulationManager::update(float deltaSeconds)
        {
            for (agents::CarAgent &car : cars_)
            {
                car.speed = ai::desiredSpeedForCar(car);

                if (car.position == car.target)
                {
                    car.target = ai::chooseNextWaypoint(rng_, waypoints_, car.position);
                }

                const float distance = advanceCar(car, deltaSeconds);
                applyOperationalCosts(car, distance);

                if (car.position == car.target)
                {
                    car.target = ai::chooseNextWaypoint(rng_, waypoints_, car.position);
                }
            }
        }

        const std::vector<agents::CarAgent> &SimulationManager::cars() const
        {
            return cars_;
        }

        const std::vector<glm::vec2> &SimulationManager::waypoints() const
        {
            return waypoints_;
        }

        const world::CityMapConfig &SimulationManager::cityMap() const
        {
            return cityMap_.config();
        }

    } // namespace systems

} // namespace civitasx
