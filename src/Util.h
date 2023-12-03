//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include <SFML/Graphics.hpp>
#include <ecs-cpp/EcsCpp.h>
#include <optional>
#include "Components.h"

using ECS = ecs::ECSManager<Circle, Verlet, ecs::EntityID, OctreeSwitch>;

using Lines = std::vector<Line>;
static constexpr float circleRadius = 1.0f;
static constexpr float queryRadius = 3.0f * circleRadius;
static constexpr int nrIterations = 4;
static constexpr int nrCircles = 20000;
static constexpr int vertexPerCircle = 3;

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
                              const sf::Vector2f &L2End);
