#pragma once

#include <random>
#include <vector>

#include <glm/vec2.hpp>

namespace civitasx
{

    struct CarAgent
    {
        int id = 0;
        glm::vec2 position{0.0f, 0.0f};
        glm::vec2 target{0.0f, 0.0f};
        float speed = 25.0f;
        float battery = 100.0f;
        float wallet = 50.0f;
    };

    class World
    {
    public:
        void initialize(unsigned int seed = 7U, int carCount = 24);
        void update(float deltaSeconds);

        const std::vector<CarAgent> &cars() const;
        const std::vector<glm::vec2> &waypoints() const;

    private:
        void assignNewTarget(CarAgent &car);

        std::mt19937 rng_;
        std::vector<CarAgent> cars_;
        std::vector<glm::vec2> waypoints_;
    };

} // namespace civitasx
