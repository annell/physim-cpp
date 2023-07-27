//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include "Components.h"
#include "SFML/Graphics.hpp"
#include "../thirdParty/ecs/include/EntityComponentSystem.h"
#include <type_traits>

namespace sf {
    class RenderWindow;
}

using ECS = ecs::ECSManager<sf::CircleShape, Circle, Verlet, ecs::EntityID>;

class RenderSystem {
public:
    struct Config {
        std::string FpsText;
        sf::RenderWindow &Window;
        sf::Text &fpsText;
        sf::Text &nrPoints;
        ECS &Ecs;
    };

    static void Run(Config &);
};

class CollisionSystem {
public:
    struct Config {
        ECS &Ecs;
        struct WorldBoundrarys &worldBoundrarys;
    };

    static void Run(Config &);

};

class GravitySystem {
public:
    struct Config {
        ECS &Ecs;
        float dt;
    };

    static void Run(Config &);
};
