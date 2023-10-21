//
// Created by Stefan Annell on 2023-07-25.
//

#include "System.h"
#include "Util.h"
#include "Physics.h"
#include <iostream>

void RenderSystem::Run(const Config &config) {
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

void GravitySystem::Run(const Config &config) {
    for (const auto &[verlet]: config.Ecs.GetSystem<Verlet>()) {
        verlet.Acceleration += {0, 9.81f};
        verlet.Velocity += verlet.Acceleration * config.dt;
        verlet.Acceleration = {0, 0};
    }
}

void ResolveCollision(ECS& ecs, ecs::EntityID id1, ecs::EntityID id2) {
    auto& verlet1 = ecs.Get<Verlet>(id1);
    auto& verlet2 = ecs.Get<Verlet>(id2);
    auto& circle1 = ecs.Get<Circle>(id1);
    auto& circle2 = ecs.Get<Circle>(id2);
    auto normal = Normalize(verlet1.Position - verlet2.Position);
    auto distance = Distance(verlet1.Position, verlet2.Position);
    auto overlap = (circle1.Radius + circle2.Radius) - distance;
    auto totalMass = circle1.Mass + circle2.Mass;
    verlet1.Position -= normal * (overlap * (circle2.Mass / totalMass));
    verlet2.Position += normal * (overlap * (circle1.Mass / totalMass));
}

void ResolveCircleLineCollision(ECS& ecs, ecs::EntityID id1, ecs::EntityID id2) {
    auto& verlet1 = ecs.Get<Verlet>(id1);
    auto& line = ecs.Get<Line>(id2);
    auto& circle = ecs.Get<Circle>(id1);
    auto result = IntersectionLineToPoint(verlet1.Position, line.Start, line.End);
    auto overlap = circle.Radius - result.distance;
    verlet1.Position += line.Normal * overlap;
}

bool FloatEqual(float a, float b) {
    return std::abs(a - b) < 0.00001;
}

bool FloatLessThan(float a, float b) {
    return a < b and not FloatEqual(a, b);
}

void CollisionSystem::Run(const Config &config) {
    auto octree = MakeOctree(config.Ecs, config.worldBoundrarys);

    float t0 = 0.0f;
    while (FloatLessThan(t0, config.dt)) {
        auto tLeft = config.dt - t0;
        CollisionResult collisionResult = { tLeft };
        for (const auto &[verlet]: config.Ecs.GetSystem<Verlet>()) {
            verlet.Update(collisionResult.tCollision);
        }

        for (const auto &[circle, verlet, id]: config.Ecs.GetSystem<Circle, Verlet, ecs::EntityID>()) {
            auto query = octree.Query(Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + 12});
            auto sphereResults = SphereCollision(config.Ecs, query, verlet, circle.Radius, id, tLeft);
            auto lineResults = LineCollision(config.Ecs, verlet, circle.Radius, id);
            collisionResult = min(sphereResults, collisionResult);
            collisionResult = min(lineResults, collisionResult);
            if (FloatEqual(collisionResult.tCollision, 0.0f)) {
                break;
            }
        }

        for (const auto &[verlet, id]: config.Ecs.GetSystem<Verlet, ecs::EntityID>()) {
            verlet.Revert();
            verlet.Update(collisionResult.tCollision);
            verlet.PreviousPosition = verlet.Position;
        }

        if (not (collisionResult.id1 and collisionResult.id2)) {
            break;
        }

        if (config.Ecs.Has<Line>(collisionResult.id2)) {
            auto& verlet1 = config.Ecs.Get<Verlet>(collisionResult.id1);
            auto radius1 = config.Ecs.Get<Circle>(collisionResult.id1).Radius;
            verlet1.Velocity -= Reflect(verlet1.Velocity, config.Ecs.Get<Line>(collisionResult.id2).Normal);
            if (collisionResult.tCollision <= 0.0000001f) {
                ResolveCircleLineCollision(config.Ecs, collisionResult.id1, collisionResult.id2);
            }
        } else {
            auto& verlet1 = config.Ecs.Get<Verlet>(collisionResult.id1);
            auto& verlet2 = config.Ecs.Get<Verlet>(collisionResult.id2);
            RecalculateSphereCollision(verlet1, verlet2);
            if (collisionResult.tCollision <= 0.0000001f) {
                ResolveCollision(config.Ecs, collisionResult.id1, collisionResult.id2);
            }
        }

        t0 += collisionResult.tCollision;
    }
}

