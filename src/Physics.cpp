#include "Physics.h"
#include <SFMLMath.hpp>

namespace {

bool FloatEqual(float a, float b) {
    return fabs(a - b) < std::numeric_limits<float>::epsilon();
}

bool FloatLessThan(float a, float b) {
    return a < b and not FloatEqual(a, b);
}

std::optional<float> VerletCircleSweep(const Verlet &A, float radiusA, const Verlet &B, float radiusB) {
    sf::Vector2f closestPoint;
    auto dist = std::abs(SegmentSegmentDistance(A.PreviousPosition, A.Position, B.PreviousPosition, B.Position, closestPoint));
    auto t = dist / (radiusA + radiusB);
    if (t <= 1.0f) {
        return t;
    }
    return std::nullopt;
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

std::optional<CollisionResult>
FirstCircleCollision(ECS &ecs, const octreeQuery &query, const Verlet &verlet, float radius, ecs::EntityID id1, float tLeft) {
    std::optional<CollisionResult> collisionResult;
    for (const auto &testPoint: query) {
        const auto &id2 = testPoint.Data;
        if (id1 == id2) {
            continue;
        }

        auto [verlet2, circle2] = ecs.GetSeveral<Verlet, Circle>(id2);
        auto sweepResults = VerletCircleSweep(verlet, radius, verlet2,
                                              circle2.Radius);
        if (!sweepResults) {
            continue;
        }

        auto t = *sweepResults * tLeft;
        if (!collisionResult) {
            collisionResult = CollisionResult{t, id1, id2, CollisionType::Circle};
        } else if (t < collisionResult->tCollision) {
            collisionResult = CollisionResult{t, id1, id2, CollisionType::Circle};
        }
    }
    return collisionResult;
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

std::optional<CollisionResult> FirstLineCollision(ECS &ecs, const Verlet &verlet, float radius, float dt) {
    std::optional<CollisionResult> collisionResult;
    for (const auto &[line, id2]: ecs.GetSystem<Line, ecs::EntityID>()) {
        float t = 1.0f;
        if (!IntersectMovingCircleLine(radius, verlet, line, t)) {
            continue;
        }
        t *= dt;
        if (!collisionResult || t <= collisionResult->tCollision) {
            collisionResult = CollisionResult{.tCollision=t, .id2=id2, .Type=CollisionType::Line};
        }
    }
    return collisionResult;
}

sf::Vector2f UpdateCircleVelocity(Verlet &A, Verlet &B) {
    auto normal = sf::getNormalized(A.Position - B.Position);

    auto a1 = sf::projection(A.Velocity, normal);
    auto a2 = sf::projection(B.Velocity, normal);

    auto optimizedP = (2.0f * (a1 - a2)) / (A.Mass + B.Mass);

    return optimizedP * B.Mass * A.Bounciness * -1.0f;
}
