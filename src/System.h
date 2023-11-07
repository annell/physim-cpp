//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include "Components.h"
#include "SFML/Graphics.hpp"
#include <ecs-cpp/EcsCpp.h>
#include <octree-cpp/OctreeCpp.h>
#include <type_traits>

namespace sf {
    class RenderWindow;
}
struct WorldBoundrarys;

struct vec {
    float x, y, z = 0;

    auto operator<=>(const vec &rhs) const = default;
};

using octreeQuery = std::vector<DataWrapper<vec, ecs::EntityID>>;
using ECS = ecs::ECSManager<sf::CircleShape, Circle, Line, Verlet, ecs::EntityID, octreeQuery>;

namespace RenderSystem {
    struct Config {
        std::string FpsText;
        sf::RenderWindow &Window;
        sf::Text &fpsText;
        sf::Text &nrPoints;
        ECS &Ecs;
        WorldBoundrarys &worldBoundrarys;
        ecs::EntityID hoveredId;
    };

    void Run(const Config &);
}

namespace ContinousCollisionSystem {
    struct Config {
        ECS &Ecs;
        WorldBoundrarys &worldBoundrarys;
        float dt = 0.0f;
    };

    void Run(const Config &);
}

namespace DiscreteCollisionSystem {
    struct Config {
        ECS &Ecs;
        WorldBoundrarys &worldBoundrarys;
        float dt = 0.0f;
    };

    void Run(const Config &);
}

namespace GravitySystem {
    struct Config {
        ECS &Ecs;
        float dt = 0.0f;
    };

    void Run(const Config &);
}

