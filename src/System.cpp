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


void CollisionSystem::Run(const Config &config) {
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
            auto query = octree.Query(Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + 12});
            collisionResult = min(SphereCollision(config.Ecs, query, verlet, circle.Radius, id, tLeft), collisionResult);
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

