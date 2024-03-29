//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include <SFML/Graphics.hpp>
#include <ecs-cpp/EcsCpp.h>
#include <optional>
#include "Components.h"
#include <future>
#include <chrono>

using ECS = ecs::ECSManager<Circle, Verlet, ecs::EntityID, octreeQuery, Line>;

using Lines = std::vector<Line>;
static constexpr float circleRadius = 1.5f;
static constexpr float queryRadius = 1.0f * circleRadius;
static constexpr int nrIterations = 2;
static constexpr int nrCircles = 40000;
static constexpr int vertexPerCircle = 3;
static constexpr bool pointRendering = true;

float RandomFloat(float min, float max);

sf::Color RandomColor();

struct WorldBoundrarys {
    sf::Vector2f Position;
    sf::Vector2f Size;

    [[nodiscard]] sf::FloatRect GetBox() const { return {Position, Size}; }
};

using Octree = OctreeCpp<sf::Vector2f, ecs::EntityID>;

Octree MakeOctree(auto &ecs, const WorldBoundrarys &worldBoundrarys) {
    Octree octree({{0,                      0},
                   {worldBoundrarys.Size.x, worldBoundrarys.Size.y}});

    for (const auto &[verlet, id]: ecs.template GetSystem<Verlet, ecs::EntityID>()) {
        if (worldBoundrarys.GetBox().contains(verlet.Position)) {
            octree.Add({{verlet.Position.x, verlet.Position.y}, id});
        }
    }

    return octree;
}

double SegmentSegmentDistance(const sf::Vector2f &L1Start, const sf::Vector2f &L1End, const sf::Vector2f &L2Start,
                              const sf::Vector2f &L2End);

template<typename R>
bool is_ready(std::future<R> const& f)
{
    return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}
