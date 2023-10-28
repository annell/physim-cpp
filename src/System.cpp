//
// Created by Stefan Annell on 2023-07-25.
//

#include "System.h"
#include "Util.h"
#include "Physics.h"
#include <iostream>
#include <cassert>

void RenderSystem::Run(const Config &config) {
    config.Window.clear();
    std::vector<ecs::EntityID> entitiesToRemove;
    for (const auto &[shape, verlet, id]: config.Ecs.GetSystem<sf::CircleShape, Verlet, ecs::EntityID>()) {
        shape.setPosition(verlet.Position);
        config.Window.draw(shape);
        sf::Text idText;
        std::string idString = "id: " + std::to_string(id.GetId()) + "\npos:{x: " + std::to_string(verlet.Position.x) + ", y: " + std::to_string(verlet.Position.y) + "}\n" + "vel:{x: " + std::to_string(verlet.Velocity.x) + ", y: " + std::to_string(verlet.Velocity.y) + "}";
        idText.setString(idString);
        idText.setFont(*config.fpsText.getFont());
        idText.setCharacterSize(12);
        idText.setPosition(verlet.Position - sf::Vector2f{30, 50.0f});
        //config.Window.draw(idText);
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
    while (FloatLessThan(t0, config.dt)) {
        auto tLeft = config.dt - t0;
        CollisionResult collisionResult = { tLeft };

        // Move all objects forward the full time period
        for (const auto &[verlet]: config.Ecs.GetSystem<Verlet>()) {
            verlet.PreviousPosition = verlet.Position;
            verlet.Update(collisionResult.tCollision);
        }

        // Check if there are any collisions in the time step
        for (const auto &[circle, verlet, id]: config.Ecs.GetSystem<Circle, Verlet, ecs::EntityID>()) {
            auto query = octree.Query(Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + 12});
            if (auto sphereResults = SphereCollision(config.Ecs, query, verlet, circle.Radius, id, tLeft)) {
                collisionResult = min(sphereResults.value(), collisionResult);
            }
            if (auto lineResults = LineCollision(config.Ecs, verlet, circle.Radius, tLeft)) {
                lineResults->id1 = id;
                collisionResult = min(lineResults.value(), collisionResult);
            }
        }

        if (!collisionResult.id1 || !collisionResult.id2) {
            break;
        }

        // Roll back and move objects to the collision point
        for (const auto &[verlet]: config.Ecs.GetSystem<Verlet>()) {
            verlet.Revert();
            verlet.Update(collisionResult.tCollision);
            verlet.PreviousPosition = verlet.Position;
        }

        if (config.Ecs.Has<Line>(collisionResult.id2)) {
            auto &verlet1 = config.Ecs.Get<Verlet>(collisionResult.id1);
            auto radius1 = config.Ecs.Get<Circle>(collisionResult.id1).Radius;
            verlet1.Velocity -= Reflect(verlet1.Velocity, config.Ecs.Get<Line>(collisionResult.id2).Normal);
            auto radius2 = config.Ecs.Get<Circle>(collisionResult.id1).Radius;
            auto line = config.Ecs.Get<Line>(collisionResult.id2);
            auto point = DistanceLineToPoint(line.Start, line.End, verlet1.Position);
            auto overlapp = radius1 - point.distance;
            if (FloatGreaterThan(overlapp, 0.0f)) {
                verlet1.Position -= line.Normal * overlapp;
            }
        } else {
            auto& verlet1 = config.Ecs.Get<Verlet>(collisionResult.id1);
            auto& verlet2 = config.Ecs.Get<Verlet>(collisionResult.id2);
            RecalculateSphereCollision(verlet1, verlet2);
            auto overlapp = config.Ecs.Get<Circle>(collisionResult.id1).Radius + config.Ecs.Get<Circle>(collisionResult.id2).Radius - Distance(verlet1.Position, verlet2.Position);
            if (FloatGreaterThan(overlapp, 0.0f)) {
                auto normal = NormalBetweenPoints(verlet1.Position, verlet2.Position);
                verlet1.Position += normal * overlapp / 2.0f;
                verlet2.Position -= normal * overlapp / 2.0f;
            }
        }

        t0 += collisionResult.tCollision;
    }
}

