//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include "SFML/Graphics.hpp"

struct Circle {
    float Radius = 10.0;
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

    void Update(float dt) {
        Position += Velocity * dt;
    }

    void Revert() {
        Position = PreviousPosition;
    }
};

