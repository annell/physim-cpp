//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include <SFML/Graphics.hpp>
#include <octree-cpp/OctreeCpp.h>
#include <ecs-cpp/EcsCpp.h>
#include <optional>
#include "Components.h"

struct vec {
    float x, y, z = 0;

    auto operator<=>(const vec &rhs) const = default;
};

using octreeQuery = std::vector<DataWrapper<vec, ecs::EntityID>>;
using ECS = ecs::ECSManager<sf::CircleShape, Circle, Line, Verlet, ecs::EntityID, octreeQuery>;
static constexpr float circleRadius = 6.0f;
static constexpr float queryRadius = 2.0f * circleRadius;
static constexpr int nrIterations = 1;
static constexpr int nrCircles = 4000;

float RandomFloat(float min, float max);

sf::Color RandomColor();

struct WorldBoundrarys {
    sf::Vector2f Position;
    sf::Vector2f Size;

    [[nodiscard]] sf::FloatRect GetBox() const { return {Position, Size}; }
};

using Octree = OctreeCpp<vec, ecs::EntityID>;

Octree MakeOctree(ECS &ecs, const WorldBoundrarys &worldBoundrarys);

double SegmentSegmentDistance(const sf::Vector2f &L1Start, const sf::Vector2f &L1End, const sf::Vector2f &L2Start,
                              const sf::Vector2f &L2End, sf::Vector2f &Out);
