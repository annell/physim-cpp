#include "Physics.h"
#include <SFMLMath.hpp>

namespace {

bool FloatEqual(float a, float b) {
    return fabs(a - b) < std::numeric_limits<float>::epsilon();
}

bool FloatLessThan(float a, float b) {
    return a < b and not FloatEqual(a, b);
}

}

const CollisionResult& Min(const CollisionResult& a, const CollisionResult& b) {
    if (FloatEqual(a.tCollision, b.tCollision) || FloatLessThan(a.tCollision, b.tCollision)) {
        return a;
    }
    return b;
}

std::optional<float> Overlapp(const sf::Vector2f &pos1, const sf::Vector2f &pos2, float radius1, float radius2) {
    float distance = sf::distance(pos1, pos2);
    if (distance < radius1 + radius2) {
        return distance / (radius1 + radius2);
    }
    return std::nullopt;
}

std::optional<float> Overlapp(const Line &line, const sf::Vector2f &position, float radius) {
    sf::Vector2f _;
    auto overlapp = radius - SegmentSegmentDistance(line.Start, line.End, position, position, _);
    if (overlapp > 0) {
        return overlapp;
    }
    return std::nullopt;
}

std::optional<float> Overlapp(const sf::CircleShape &circle1, const sf::CircleShape &circle2) {
    return Overlapp(circle1.getPosition(), circle2.getPosition(), circle1.getRadius(), circle2.getRadius());
}

bool IntersectMovingCircleLine(float radius, const Verlet &verlet, const Line &line, float &u0) {
    sf::Vector2f closestPoint;
    auto dist = std::abs(SegmentSegmentDistance(verlet.PreviousPosition, verlet.Position, line.Start, line.End, closestPoint));

    if (dist >= radius) {
        u0 = 1.0f;
        return false;
    } else {
        u0 = sf::distance(verlet.PreviousPosition, closestPoint) / sf::distance(verlet.PreviousPosition, verlet.Position);
        return true;
    }
}

sf::Vector2f UpdateCircleVelocity(Verlet &A, Verlet &B) {
    auto normal = sf::getNormalized(A.Position - B.Position);

    auto a1 = sf::projection(A.Velocity, normal);
    auto a2 = sf::projection(B.Velocity, normal);

    auto optimizedP = (2.0f * (a1 - a2)) / (A.Mass + B.Mass);

    return optimizedP * B.Mass * A.Bounciness * -1.0f;
}
