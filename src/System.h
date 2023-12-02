//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include "Util.h"
#include "Physics.h"
#include <type_traits>

namespace sf {
    class RenderWindow;
}
struct WorldBoundrarys;


namespace RenderSystem {
    struct Config {
        std::string FpsText;
        sf::RenderWindow &Window;
        sf::Text &fpsText;
        sf::Text &nrPoints;
        ECS &Ecs;
        WorldBoundrarys &worldBoundrarys;
        ecs::EntityID hoveredId;
        Lines& lines;
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
        Lines &Lines;
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

