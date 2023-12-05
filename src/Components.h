//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include "SFML/Graphics.hpp"
#include <SFMLMath.hpp>
#include <octree-cpp/OctreeCpp.h>

struct Circle {
    float Radius = 10.0;
    sf::Color Color = sf::Color::Cyan;
};

struct Line {
    sf::Vector2f Start;
    sf::Vector2f End;
    sf::Vector2f Normal;
    float d = 0.0;
};

struct Verlet {
    sf::Vector2f Position;
    sf::Vector2f Acceleration;
    sf::Vector2f Velocity;
    sf::Vector2f PreviousPosition;
    float Mass = 1.0;
    float Bounciness = 0.9f;
    float Friction = 0.5;

    void MaxVelocity(float max) {
        if (sf::getLength(Velocity) > max) {
            Velocity = sf::getNormalized(Velocity) * max;
        }
    }

    void Update(float dt) {
        MaxVelocity(100.0f);
        Position += Velocity * dt;
    }
};

struct vec {
    float x, y, z = 0;

    auto operator<=>(const vec &rhs) const = default;
};

using octreeQuery = std::vector<DataWrapper<vec, ecs::EntityID>>;
struct OctreeSwitch {
    bool UpdatingOne = false;
    octreeQuery Query1;
    octreeQuery Query2;

    const octreeQuery& GetUsableQuery() const {
        if (UpdatingOne) {
            return Query2;
        }
        return Query1;
    }

    octreeQuery& GetUpdatingQuery() {
        if (UpdatingOne) {
            return Query1;
        }
        return Query2;
    }

    void Switch() {
        UpdatingOne = !UpdatingOne;
    }
};

