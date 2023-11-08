#include "Physics.h"
#include <SFMLMath.hpp>

bool FloatEqual(float a, float b) {
    return fabs(a - b) < std::numeric_limits<float>::epsilon();
}

bool FloatLessThan(float a, float b) {
    return a < b and not FloatEqual(a, b);
}

CollisionResult min(CollisionResult a, CollisionResult b) {
    if (FloatEqual(a.tCollision, b.tCollision) || FloatLessThan(a.tCollision, b.tCollision)) {
        return a;
    }
    return b;
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

std::optional<CollisionResult> LineCollisionSweep(ECS &ecs, const Verlet &verlet, float radius, float dt) {
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

const void UpdateCircleVelocity(Verlet &A, Verlet &B) {
    auto normal = sf::getNormalized(A.Position - B.Position);

    auto a1 = sf::projection(A.Velocity, normal);
    auto a2 = sf::projection(B.Velocity, normal);

    auto optimizedP = (2.0f * (a1 - a2)) / (A.Mass + B.Mass);

    A.Velocity -= optimizedP * B.Mass * A.Bounciness;
    B.Velocity += optimizedP * A.Mass * B.Bounciness;
}
