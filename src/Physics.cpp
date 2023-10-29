#include "Physics.h"

CollisionResult min(CollisionResult a, CollisionResult b) {
    if (FloatEqual(a.tCollision, b.tCollision) || FloatLessThan(a.tCollision, b.tCollision)) {
        return a;
    }
    return b;
}

const bool SphereSphereSweep
        (
                const float ra, //radius of sphere A
                const sf::Vector2f &A0, //Current pos sphere A
                const sf::Vector2f &A1, //Next pos sphere A
                const float rb, //radius of sphere B
                const sf::Vector2f &B0, //Current pos sphere B
                const sf::Vector2f &B1, //Next pos sphere B
                float &u0 //normalized time of first collision
        ) {
    sf::Vector2f closestPoint;
    auto dist = SegmentSegmentDistance(A0, A1, B0, B1, closestPoint);

    auto Avel = A0 - A1;
    auto Bvel = B0 - B1;
    auto B = Avel - Bvel;

    auto point = DistanceLineToPoint(B0, B, closestPoint);

    u0 = point.t;
    return dist <= ra + rb;
}

const bool VerletSphereSweep(const Verlet& A, float radiusA, const Verlet& B, float radiusB, float& u0) {
    return SphereSphereSweep(radiusA, A.PreviousPosition, A.Position, radiusB, B.PreviousPosition, B.Position, u0);
}

bool IntersectMovingSpherePlane(float radius, const Verlet& verlet, const Line& line, float &u0) {
    // Compute distance of sphere center to plane
    sf::Vector2f closestPoint;
    auto dist = SegmentSegmentDistance(verlet.PreviousPosition, verlet.Position, line.Start, line.End, closestPoint);

    if (std::abs(dist) >= radius) {
        u0 = 1.0f;
        return false;
    } else {
        u0 = Distance(verlet.PreviousPosition, closestPoint) / Distance(verlet.PreviousPosition, verlet.Position);
        return true;
    }
}

std::optional<CollisionResult> LineCollision(ECS& ecs, const Verlet& verlet, float radius, float dt) {
    std::optional<CollisionResult> collisionResult;
    for (const auto &[line, id2]: ecs.GetSystem<Line, ecs::EntityID>()) {
        float t = 1.0f;
        if (IntersectMovingSpherePlane(radius, verlet, line, t)) {
            t *= dt;
            if (!collisionResult) {
                collisionResult = CollisionResult{.tCollision=t, .id2=id2};
            } else if (t <= collisionResult->tCollision) {
                collisionResult = CollisionResult{.tCollision=t, .id2=id2};
            }
        }
    }
    return collisionResult;
}

const void RecalculateSphereCollision(Verlet& A, Verlet& B) {
    auto normal = Normalize(A.Position - B.Position);

    auto a1 = Projection(A.Velocity, normal);
    auto a2 = Projection(B.Velocity, normal);

    auto optimizedP = (2.0f * (a1 - a2)) / (A.Mass + B.Mass);

    A.Velocity -= optimizedP * B.Mass;
    B.Velocity += optimizedP * A.Mass;
}
