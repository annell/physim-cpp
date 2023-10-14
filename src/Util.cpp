#include "Util.h"

#include "SFML/Graphics.hpp"
#include <random>

float RandomFloat(float min, float max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min, max);
    return dis(gen);
}

sf::Color RandomColor() {
    return sf::Color(RandomFloat(0, 255), RandomFloat(0, 255), RandomFloat(0, 255));
}

bool Collision(const sf::CircleShape &circle1, const sf::CircleShape &circle2) {
    float distance = Distance(circle1.getPosition(), circle2.getPosition());
    return distance < circle1.getRadius() + circle2.getRadius();
}

float Distance(const sf::Vector2f &point1, const sf::Vector2f &point2) {
    float dx = point2.x - point1.x;
    float dy = point2.y - point1.y;
    return std::sqrt(dx * dx + dy * dy);
}

float Dot(const sf::Vector2f &v1, const sf::Vector2f &v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

sf::Vector2f NormalBetweenPoints(const sf::Vector2f &point1, const sf::Vector2f &point2) {
    sf::Vector2f direction = point2 - point1;
    sf::Vector2f normal = {direction.y, -direction.x};
    normal /= std::sqrt(normal.x * normal.x + normal.y * normal.y);
    return normal;
}

void Reflect(sf::Vector2f &vector, const sf::Vector2f &normal) {
    vector -= 2 * Dot(vector, normal) * normal;
}

sf::Vector2f Normalize(const sf::Vector2f &vector) {
    float length = std::sqrt(vector.x * vector.x + vector.y * vector.y);
    if (length != 0) {
        return vector / length;
    }
    return vector;
}

sf::Vector2f Projection(const sf::Vector2f &vector, const sf::Vector2f &axis) {
    float k = Dot(vector, axis) / Dot(axis, axis);
    return k * axis;
}

float Hypot2(const sf::Vector2f &v1, const sf::Vector2f &v2) {
    return Dot(v1 - v2, v1 - v2);
}


Octree MakeOctree(ECS& ecs, const WorldBoundrarys& worldBoundrarys) {
    Octree octree({{0,                             0,                             0},
                   {worldBoundrarys.Size.x, worldBoundrarys.Size.y, 0}});

    for (auto &it: ecs) {
        if (ecs.Has<Verlet>(it.id)) {
            auto& verlet = ecs.Get<Verlet>(it.id);
            if (worldBoundrarys.GetBox().contains(verlet.Position)) {
                octree.Add({{verlet.Position.x, verlet.Position.y}, it.id});
            }
        }
    }

    return std::move(octree);
}
IntersectionResult IntersectionLineToPoint(const sf::Vector2f& A, const sf::Vector2f& B, const sf::Vector2f& C) {
    auto AC = C - A;
    auto AB = B - A;

    auto D = Projection(AC, AB) + A;
    auto AD = D - A;

    auto k = AB.x > AB.y ? AD.x / AB.x : AD.y / AB.y;

    if (k <= 0) {
        return {std::sqrt(Hypot2(C, A)), 0.0f};
    } else if (k >= 1) {
        return {std::sqrt(Hypot2(C, B)), 1.0f};
    }
    return {std::sqrt(Hypot2(C, D)), D.x / (A.x + B.x)};
}
