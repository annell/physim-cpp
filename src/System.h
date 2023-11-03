//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include "Components.h"
#include "SFML/Graphics.hpp"
#include <EcsCpp.h>
#include <type_traits>

namespace sf {
    class RenderWindow;
}
struct WorldBoundrarys;

using ECS = ecs::ECSManager<sf::CircleShape, Circle, Line, Verlet, ecs::EntityID>;

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

namespace CollisionSystem {
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

