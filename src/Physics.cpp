#include "Physics.h"

CollisionResult min(CollisionResult a, CollisionResult b) {
    if (a.tCollision < b.tCollision) {
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
    if (Distance(A0, B0) < ra + rb) {
        u0 = 0.0f;
        return true;
    }
    auto Avel = A0 - A1;
    auto Bvel = B0 - B1;
    auto B = Avel - Bvel;

    auto res = IntersectionLineToPoint(B0, B, A0);

    if (res.distance > ra + rb) {
        return false;
    }
    u0 = res.t;
    return true;
}

const bool VerletSphereSweep(const Verlet& A, float radiusA, const Verlet& B, float radiusB, float& u0) {
    return SphereSphereSweep(radiusA, A.PreviousPosition, A.Position, radiusB, B.PreviousPosition, B.Position, u0);
}

bool IntersectMovingSpherePlane(float radius, const Verlet& verlet, const Line& line, float &u0) {
    // Compute distance of sphere center to plane
    auto d = IntersectionLineToPoint(line.Start, line.End, {0, 0}).distance;
    float dist = Dot(line.Normal, verlet.Position) - d;
    if (std::abs(dist) <= radius) {
        // The sphere is already overlapping the plane. Set time of
        // intersection to zero and q to sphere center
        u0 = 0.0f;
        return true;
    } else {
        float denom = Dot(line.Normal, verlet.Velocity);
        if (denom * dist >= 0.0f) {
            // No intersection as sphere moving parallel to or away from plane
            return false;
        } else {
            // Sphere is moving towards the plane
            // Use +r in computations if sphere in front of plane, else -r
            float r = dist > 0.0f ? radius : -radius;
            u0 = (r - dist) / denom;
            return true;
        }
    }
}

CollisionResult LineCollision(ECS& ecs, const Verlet& verlet, float radius, ecs::EntityID id1) {
    CollisionResult collisionResult;
    for (const auto &[line, id2]: ecs.GetSystem<Line, ecs::EntityID>()) {
        float t = 0.0f;
        if (IntersectMovingSpherePlane(radius, verlet, line, t)) {
            if (t <= collisionResult.tCollision) {
                collisionResult.tCollision = t;
                collisionResult.id1 = id1;
                collisionResult.id2 = id2;
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
