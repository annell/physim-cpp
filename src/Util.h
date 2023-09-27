//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include "SFML/Graphics.hpp"
#include "System.h"
#include "../thirdParty/octree-cpp/src/OctreeCpp.h"

float RandomFloat(float min, float max);

sf::Color RandomColor();

bool Collision(const sf::CircleShape &circle1, const sf::CircleShape &circle2);

float Distance(const sf::Vector2f &point1, const sf::Vector2f &point2);

float Dot(const sf::Vector2f &v1, const sf::Vector2f &v2);

float Hypot2(const sf::Vector2f &v1, const sf::Vector2f &v2);

sf::Vector2f Projection(const sf::Vector2f &vector, const sf::Vector2f &axis);

sf::Vector2f Normalize(const sf::Vector2f &vector);

sf::Vector2f NormalBetweenPoints(const sf::Vector2f &point1, const sf::Vector2f &point2);

void Reflect(sf::Vector2f &vector, const sf::Vector2f &normal);

struct vec {
  float x, y, z = 0;

  auto operator<=>(const vec &rhs) const = default;
};

struct WorldBoundrarys {
  sf::Vector2f Position;
  sf::Vector2f Size;

  [[nodiscard]] sf::FloatRect GetBox() const { return {Position, Size}; }
};

using Octree = OctreeCpp<vec, ecs::EntityID>;

Octree MakeOctree(ECS& ecs, const WorldBoundrarys& worldBoundrarys);

struct IntersectionResult {
    float distance = 0.0f;
    float t = 0.0f; //Normalized time of closest point between Line and circle
};
IntersectionResult IntersectionLineToPoint(const sf::Vector2f& A, const sf::Vector2f& B, const sf::Vector2f& C);
