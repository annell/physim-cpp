//
// Created by Stefan Annell on 2023-07-25.
//
#pragma once

#include "Util.h"
#include "Components.h"
#include "../thirdParty/ecs-cpp/src/EcsCpp.h"
#include "../thirdParty/octree-cpp/src/OctreeCpp.h"
#include <optional>

struct CollisionResult {
    float tCollision = 100.0f;
    ecs::EntityID id1;
    ecs::EntityID id2;
};


CollisionResult min(CollisionResult a, CollisionResult b);
const bool SphereSphereSweep
        (
                const float ra, //radius of sphere A
                const sf::Vector2f &A0, //previous position of sphere A
                const sf::Vector2f &A1, //current position of sphere A
                const float rb, //radius of sphere B
                const sf::Vector2f &B0, //previous position of sphere B
                const sf::Vector2f &B1, //current position of sphere B
                float &u0 //normalized time of first collision
        );
const bool VerletSphereSweep(const Verlet& A, float radiusA, const Verlet& B, float radiusB, float& u0);
std::optional<CollisionResult> SphereCollision(ECS& ecs, const auto& query, const Verlet& verlet, float radius, ecs::EntityID id1, float tLeft) {
    std::optional<CollisionResult> collisionResult;
    for (const auto &testPoint: query) {
        const auto& id2 = testPoint.Data;
        if (id1 != id2) {
            float normalizedCollisionT = 1.0f;

            auto& verlet2 = ecs.Get<Verlet>(id2);
            const auto radius2 = ecs.Get<Circle>(id2).Radius;
            if (VerletSphereSweep(verlet, radius, verlet2,
                                  radius2, normalizedCollisionT)) {
                auto t = normalizedCollisionT * tLeft;
                if (!collisionResult) {
                    collisionResult = CollisionResult{t, id1, id2};
                } else if (t <= collisionResult->tCollision) {
                    collisionResult = CollisionResult{t, id1, id2};
                }
            }
        }
    }
    return collisionResult;
}
bool IntersectMovingSpherePlane(float radius, const Verlet& verlet, const Line& line, float &u0);
std::optional<CollisionResult> LineCollision(ECS& ecs, const Verlet& verlet, float radius, float dt);
const void RecalculateSphereCollision(Verlet& A, Verlet& B);
