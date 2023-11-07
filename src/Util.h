//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include "System.h"
#include <octree-cpp/OctreeCpp.h>

float RandomFloat(float min, float max);

sf::Color RandomColor();

bool Collision(const class sf::CircleShape &circle1, const sf::CircleShape &circle2);

sf::Vector2f NormalBetweenPoints(const sf::Vector2f &point1, const sf::Vector2f &point2);

bool FloatEqual(float a, float b);

bool FloatIsZero(float a);

bool FloatLessThan(float a, float b);

bool FloatGreaterThan(float a, float b);

struct WorldBoundrarys {
    sf::Vector2f Position;
    sf::Vector2f Size;

    [[nodiscard]] sf::FloatRect GetBox() const { return {Position, Size}; }
};

using Octree = OctreeCpp<vec, ecs::EntityID>;

Octree MakeOctree(ECS &ecs, const WorldBoundrarys &worldBoundrarys);

struct IntersectionResult {
    float distance = 0.0f;
    float t = 0.0f; //Normalized time of closest point between Line and circle
};

double SegmentSegmentDistance(const sf::Vector2f &L1Start, const sf::Vector2f &L1End, const sf::Vector2f &L2Start,
                              const sf::Vector2f &L2End, sf::Vector2f &Out);

float vectorAngle(float x, float y);