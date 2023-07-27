//
// Created by Stefan Annell on 2023-07-25.
//

#pragma once

#include "SFML/Graphics.hpp"

float RandomFloat(float min, float max);

sf::Color RandomColor();

float Distance(const sf::Vector2f &point1, const sf::Vector2f &point2);

float Dot(const sf::Vector2f &v1, const sf::Vector2f &v2);

sf::Vector2f Normalize(const sf::Vector2f &vector);

struct vec {
  float x, y, z = 0;

  auto operator<=>(const vec &rhs) const = default;
};

struct WorldBoundrarys {
  sf::Vector2f Position;
  sf::Vector2f Size;

  [[nodiscard]] sf::FloatRect GetBox() const { return {Position, Size}; }
};
