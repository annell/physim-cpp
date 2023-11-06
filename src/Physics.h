//
// Created by Stefan Annell on 2023-07-25.
//
#pragma once

#include "Util.h"
#include "Components.h"
#include <ecs-cpp/EcsCpp.h>
#include <octree-cpp/OctreeCpp.h>
#include <optional>

enum class CollisionType {
    Circle,
    Line
};

struct CollisionResult {
    float tCollision = 100.0f;
    ecs::EntityID id1;
    ecs::EntityID id2;

    std::optional<CollisionType> Type;
};


CollisionResult min(CollisionResult a, CollisionResult b);

std::optional<float> CircleCircleSweep
        (
                const float ra, //radius of sphere A
                const sf::Vector2f &A0, //previous position of sphere A
                const sf::Vector2f &A1, //current position of sphere A
                const float rb, //radius of sphere B
                const sf::Vector2f &B0, //previous position of sphere B
                const sf::Vector2f &B1 //current position of sphere B
        );

std::optional<float> VerletCircleSweep(const Verlet &A, float radiusA, const Verlet &B, float radiusB);

std::optional<CollisionResult>
CircleCollision(ECS &ecs, const auto &query, const Verlet &verlet, float radius, ecs::EntityID id1, float tLeft) {
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

bool IntersectMovingCircleLine(float radius, const Verlet &verlet, const Line &line, float &u0);

std::optional<CollisionResult> LineCollision(ECS &ecs, const Verlet &verlet, float radius, float dt);

const void RecalculateCircleCollision(Verlet &A, Verlet &B);
