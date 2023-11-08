//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include "System.h"
#include <octree-cpp/OctreeCpp.h>

float RandomFloat(float min, float max);

sf::Color RandomColor();

std::optional<float> Overlapp(const sf::Vector2f &pos1, const sf::Vector2f &pos2, float radius1, float radius2);

std::optional<float> Overlapp(const Line &l1, const sf::Vector2f &pos, float radius1);

std::optional<float> Overlapp(const sf::CircleShape &circle1, const sf::CircleShape &circle2);

struct WorldBoundrarys {
    sf::Vector2f Position;
    sf::Vector2f Size;

    [[nodiscard]] sf::FloatRect GetBox() const { return {Position, Size}; }
};

using Octree = OctreeCpp<vec, ecs::EntityID>;

Octree MakeOctree(ECS &ecs, const WorldBoundrarys &worldBoundrarys);

double SegmentSegmentDistance(const sf::Vector2f &L1Start, const sf::Vector2f &L1End, const sf::Vector2f &L2Start,
                              const sf::Vector2f &L2End, sf::Vector2f &Out);
