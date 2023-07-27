//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include "SFML/Graphics.hpp"

struct Circle {
    float Radius = 10.0;
    float Mass = 1.0;
    float Bounciness = 0.5;
    float Friction = 0.5;
};

struct Verlet {
    sf::Vector2f Position;
    sf::Vector2f Acceleration;
    sf::Vector2f Velocity;
    sf::Vector2f PreviousPosition;
    float MaxVelocity = 1.0;

    void CapVelocity() {
        if (Velocity.x > MaxVelocity) {
            Velocity.x = MaxVelocity;
        }
        if (Velocity.x < -MaxVelocity) {
            Velocity.x = -MaxVelocity;
        }
        if (Velocity.y > MaxVelocity) {
            Velocity.y = MaxVelocity;
        }
        if (Velocity.y < -MaxVelocity) {
            Velocity.y = -MaxVelocity;
        }
    }

    void Update(float dt) {
        PreviousPosition = Position;
        Velocity += Acceleration * dt * dt;
        CapVelocity();
        Position += Velocity;
        Acceleration = {0, 0};
    }
};

