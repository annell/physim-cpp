//
// Created by Stefan Annell on 2023-07-25.
//
#pragma once

#include "Util.h"

enum class CollisionType {
    Circle,
    Line
};

struct CollisionResult {
    float tCollision = 100.0f;
    ecs::EntityID id1;
    ecs::EntityID id2;

    std::optional<CollisionType> Type;
};


const CollisionResult& Min(const CollisionResult& a, const CollisionResult& b);

std::optional<float> Overlapp(const sf::Vector2f &pos1, const sf::Vector2f &pos2, float radius1, float radius2);

std::optional<float> Overlapp(const Line &l1, const sf::Vector2f &pos, float radius1);

std::optional<float> Overlapp(const sf::CircleShape &circle1, const sf::CircleShape &circle2);

bool IntersectMovingCircleLine(float radius, const Verlet &verlet, const Line &line);

sf::Vector2f UpdateCircleVelocity(Verlet &A, Verlet &B);
