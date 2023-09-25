//
// Created by Stefan Annell on 2023-07-25.
//

#include "System.h"
#include "Util.h"
#include "Physics.h"
#include <iostream>

const bool VerletSphereSweep(const Verlet& A, float radiusA, const Verlet& B, float radiusB, float& u0);
const void RecalculateSphereCollision(Verlet& A, float radiusA, Verlet& B, float radiusB);

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

const bool SphereSphereSweep
        (
                const float ra, //radius of sphere A
                const sf::Vector2f &A0, //previous position of sphere A
                const sf::Vector2f &A1, //current position of sphere A
                const float rb, //radius of sphere B
                const sf::Vector2f &B0, //previous position of sphere B
                const sf::Vector2f &B1, //current position of sphere B
                float &u0 //normalized time of first collision
        ) {
    if (A0 == A1) {
        return false;
    }
    auto Bvel = B0 - B1;
    auto B = A1 - Bvel;

    auto res = IntersectionLineToPoint(A0, B, B0);

    if (res.distance > ra + rb) {
        return false;
    }
    u0 = res.t;
    return true;
}

void RenderSystem::Run(Config &config) {
    config.Window.clear();
    std::vector<ecs::EntityID> entitiesToRemove;
    for (const auto &[shape, verlet, id]: config.Ecs.GetSystem<sf::CircleShape, Verlet, ecs::EntityID>()) {
        shape.setPosition(verlet.Position);
        config.Window.draw(shape);
        if (!config.worldBoundrarys.GetBox().contains(verlet.Position)) {
            entitiesToRemove.push_back(id);
        }
    }

    config.fpsText.setString(config.FpsText);
    config.nrPoints.setString(std::to_string(config.Ecs.Size()));
    config.Window.draw(config.fpsText);
    config.Window.draw(config.nrPoints);
    config.Window.display();

    for (const auto &id: entitiesToRemove) {
        config.Ecs.RemoveEntity(id);
    }
}

void GravitySystem::Run(Config &config) {
    for (const auto &[verlet]: config.Ecs.GetSystem<Verlet>()) {
        verlet.Acceleration += {0, 9.81f};
        verlet.Velocity += verlet.Acceleration * config.dt;
        verlet.Acceleration = {0, 0};
    }
}

struct CollisionResult {
    float tCollision = 100.0f;
    ecs::EntityID id1;
    ecs::EntityID id2;
};

CollisionResult SphereCollision(ECS& ecs, const Octree& octree, const Verlet& verlet, float radius, ecs::EntityID id1, float tLeft) {
    CollisionResult collisionResult;
    auto query = octree.Query(Octree::Sphere{{verlet.Position.x, verlet.Position.y}, radius + 12});
    for (const auto &testPoint: query) {
        const auto& id2 = testPoint.Data;
        if (id1 != id2) {
            float normalizedCollisionT = 0.0f;

            auto& verlet2 = ecs.Get<Verlet>(id2);
            const auto radius2 = ecs.Get<Circle>(id2).Radius;
            if (VerletSphereSweep(verlet, radius, verlet2,
                                  radius2, normalizedCollisionT)) {
                auto t = normalizedCollisionT * tLeft;
                if (t <= collisionResult.tCollision) {
                    collisionResult.tCollision = t;
                    collisionResult.id1 = id1;
                    collisionResult.id2 = id2;
                }
            }
        }
    }
    return collisionResult;
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

CollisionResult min(CollisionResult a, CollisionResult b) {
    if (a.tCollision < b.tCollision) {
        return a;
    }
    return b;
}

void CollisionSystem::Run(Config &config) {
    auto octree = MakeOctree(config.Ecs, config.worldBoundrarys);

    float t0 = 0.0f;
    bool firstCollision = true;
    while (t0 < config.dt) {
        auto tLeft = config.dt - t0;
        CollisionResult collisionResult = { tLeft };
        for (const auto &[verlet]: config.Ecs.GetSystem<Verlet>()) {
            verlet.Update(collisionResult.tCollision);
        }

        for (const auto &[circle, verlet, id]: config.Ecs.GetSystem<Circle, Verlet, ecs::EntityID>()) {
            collisionResult = min(SphereCollision(config.Ecs, octree, verlet, circle.Radius, id, tLeft), collisionResult);
            collisionResult = min(LineCollision(config.Ecs, verlet, circle.Radius, id), collisionResult);
        }

        for (const auto &[verlet, id]: config.Ecs.GetSystem<Verlet, ecs::EntityID>()) {
            verlet.Revert();
            verlet.Update(collisionResult.tCollision);
            verlet.PreviousPosition = verlet.Position;
        }

        if (not (collisionResult.id1 and collisionResult.id2)) {
            break;
        }
        firstCollision = false;

        if (config.Ecs.Has<Line>(collisionResult.id2)) {
            auto& verlet1 = config.Ecs.Get<Verlet>(collisionResult.id1);
            auto radius1 = config.Ecs.Get<Circle>(collisionResult.id1).Radius;
            Reflect(verlet1.Velocity, config.Ecs.Get<Line>(collisionResult.id2).Normal);
        } else {
            auto& verlet1 = config.Ecs.Get<Verlet>(collisionResult.id1);
            auto radius1 = config.Ecs.Get<Circle>(collisionResult.id1).Radius;

            auto& verlet2 = config.Ecs.Get<Verlet>(collisionResult.id2);
            const auto radius2 = config.Ecs.Get<Circle>(collisionResult.id2).Radius;

            RecalculateSphereCollision(verlet1, radius1, verlet2, radius2);
        }
        if (collisionResult.tCollision <= 0.001) {
            collisionResult.tCollision = 0.001;
        }
        t0 += collisionResult.tCollision;
    }
}

void BoundraryCollisionSystem::Run(Config &config) {
    for (const auto &[circle, verlet]: config.Ecs.GetSystem<Circle, Verlet>()) {
        HandleCircleBoxCollision(verlet, config.worldBoundrarys.GetBox(), circle.Radius);
    }
}


const bool VerletSphereSweep(const Verlet& A, float radiusA, const Verlet& B, float radiusB, float& u0) {
    return SphereSphereSweep(radiusA, A.PreviousPosition, A.Position, radiusB, B.PreviousPosition, B.Position, u0);
}

const void RecalculateSphereCollision(Verlet& A, float radiusA, Verlet& B, float radiusB) {
    // Calculate the collision normal
    auto n = NormalBetweenPoints(A.Position, B.Position);
    auto n2 = NormalBetweenPoints(B.Position, A.Position);

    //Relative velocity
    auto rv = B.Velocity - A.Velocity;
    rv = rv * 0.5f;

    // Update both velocities after collision
    A.Velocity = rv;
    B.Velocity = -rv;
    Reflect(A.Velocity, n);
    Reflect(B.Velocity, n2);
}
