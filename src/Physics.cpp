#include "Physics.h"

CollisionResult min(CollisionResult a, CollisionResult b) {
    if (FloatEqual(a.tCollision, b.tCollision) || FloatLessThan(a.tCollision, b.tCollision)) {
        return a;
    }
    return b;
}

std::optional<float> CircleCircleSweep
        (
                const float ra, //radius of sphere A
                const sf::Vector2f &A0, //Current pos sphere A
                const sf::Vector2f &A1, //Next pos sphere A
                const float rb, //radius of sphere B
                const sf::Vector2f &B0, //Current pos sphere B
                const sf::Vector2f &B1 //Next pos sphere B
        ) {
    sf::Vector2f closestPoint;
    auto dist = std::abs(SegmentSegmentDistance(A0, A1, B0, B1, closestPoint));
    if (dist <= ra + rb) {
        return dist / (ra + rb);
    }
    return std::nullopt;
}

std::optional<float> VerletCircleSweep(const Verlet &A, float radiusA, const Verlet &B, float radiusB) {
    return CircleCircleSweep(radiusA, A.PreviousPosition, A.Position, radiusB, B.PreviousPosition, B.Position);
}

bool IntersectMovingCircleLine(float radius, const Verlet &verlet, const Line &line, float &u0) {
    sf::Vector2f closestPoint;
    auto dist = std::abs(SegmentSegmentDistance(verlet.PreviousPosition, verlet.Position, line.Start, line.End, closestPoint));

    if (dist >= radius) {
        u0 = 1.0f;
        return false;
    } else {
        u0 = Distance(verlet.PreviousPosition, closestPoint) / Distance(verlet.PreviousPosition, verlet.Position);
        return true;
    }
}

std::optional<CollisionResult> LineCollision(ECS &ecs, const Verlet &verlet, float radius, float dt) {
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

const void RecalculateCircleCollision(Verlet &A, Verlet &B) {
    auto normal = Normalize(A.Position - B.Position);

    auto a1 = Projection(A.Velocity, normal);
    auto a2 = Projection(B.Velocity, normal);

    auto optimizedP = (2.0f * (a1 - a2)) / (A.Mass + B.Mass);

    A.Velocity -= optimizedP * B.Mass * B.Bounciness;
    B.Velocity += optimizedP * A.Mass * A.Bounciness;
}
