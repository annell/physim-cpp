//
// Created by Stefan Annell on 2023-07-25.
//

#include "System.h"
#include "Util.h"
#include "Physics.h"
#include <iostream>
#include <cassert>

static constexpr float queryRadius = 30.0f;

void RenderSystem::Run(const Config &config) {
    config.Window.clear();
    std::vector<ecs::EntityID> entitiesToRemove;
    std::optional<sf::Vector2f> hoveredPos = config.hoveredId ? config.Ecs.Get<Verlet>(config.hoveredId).Position
                                                              : std::optional<sf::Vector2f>();
    for (const auto &[shape, verlet, id]: config.Ecs.GetSystem<sf::CircleShape, Verlet, ecs::EntityID>()) {
        sf::RectangleShape line;
        line.setSize({2, shape.getRadius()});
        line.setPosition(verlet.Position - sf::Vector2f{1, 0});
        line.setRotation(vectorAngle(verlet.Velocity.x, verlet.Velocity.y) - 90);
        line.setFillColor(sf::Color::Magenta);
        shape.setPosition(verlet.Position);
        if (id == config.hoveredId) {
            shape.setFillColor(sf::Color::Red);
        } else {
            if (hoveredPos) {
                auto distance = Distance(verlet.Position, *hoveredPos);
                if (distance <= shape.getRadius() + queryRadius) {
                    shape.setFillColor(sf::Color::Green);
                } else {
                    shape.setFillColor(sf::Color::Cyan);
                }
            } else {
                shape.setFillColor(sf::Color::Cyan);
            }
        }
        config.Window.draw(shape);
        config.Window.draw(line);
        if (id == config.hoveredId) {
            sf::CircleShape outline;
            auto radius = shape.getRadius();
            outline.setRadius(radius + queryRadius);
            outline.setOrigin(radius + queryRadius, radius + queryRadius);
            outline.setPosition(verlet.Position);
            outline.setFillColor(sf::Color::Transparent);
            outline.setOutlineColor(sf::Color::Red);
            outline.setOutlineThickness(2);

            config.Window.draw(outline);
            auto octree = MakeOctree(config.Ecs, config.worldBoundrarys);
            auto query = octree.Query(Octree::Sphere{{verlet.Position.x, verlet.Position.y}, radius + queryRadius});
            std::optional<float> minDistance;
            std::optional<float> minOverlapp;
            for (const auto &testPoint: query) {
                const auto &id2 = testPoint.Data;
                if (id != id2) {
                    auto &shape = config.Ecs.Get<sf::CircleShape>(id2);
                    shape.setFillColor(sf::Color::Green);
                    auto &verlet2 = config.Ecs.Get<Verlet>(id2);
                    // Draw line between the two points
                    sf::Vertex line[] = {
                            sf::Vertex(verlet.Position),
                            sf::Vertex(verlet2.Position)
                    };
                    config.Window.draw(line, 2, sf::Lines);
                    minDistance = std::min(minOverlapp.value_or(1000.0f), Distance(verlet.Position, verlet2.Position));
                    minOverlapp = std::min(minOverlapp.value_or(1000.0f),
                                           Distance(verlet.Position, verlet2.Position) - (radius + shape.getRadius()));
                }
            }
            sf::Text idText;
            std::string idString = "id: " + std::to_string(id.GetId()) + "\ndistance: " +
                                   (minDistance ? std::to_string(*minDistance) : "nan") + "\noverlapp: " +
                                   (minOverlapp ? std::to_string(*minOverlapp) : "nan") + "\npos:{x: " +
                                   std::to_string(verlet.Position.x) + ", y: " + std::to_string(verlet.Position.y) +
                                   "}\n" + "vel:{x: " + std::to_string(verlet.Velocity.x) + ", y: " +
                                   std::to_string(verlet.Velocity.y) + "}";
            idText.setString(idString);
            idText.setFont(*config.fpsText.getFont());
            idText.setCharacterSize(15);
            idText.setPosition(verlet.Position - sf::Vector2f{50, 150.0f});
            config.Window.draw(idText);
        }

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
        CollisionResult collisionResult = {tLeft};

        // Move all objects forward the full time period
        for (const auto &[verlet]: config.Ecs.GetSystem<Verlet>()) {
            verlet.PreviousPosition = verlet.Position;
            verlet.Update(collisionResult.tCollision);
        }

        // Check if there are any collisions in the time step
        for (const auto &[circle, verlet, id]: config.Ecs.GetSystem<Circle, Verlet, ecs::EntityID>()) {
            auto query = octree.Query(
                    Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + queryRadius});
            if (auto sphereResults = SphereCollision(config.Ecs, query, verlet, circle.Radius, id, tLeft)) {
                collisionResult = min(sphereResults.value(), collisionResult);
            }
            if (auto lineResults = LineCollision(config.Ecs, verlet, circle.Radius, tLeft)) {
                lineResults->id1 = id;
                collisionResult = min(lineResults.value(), collisionResult);
            }
        }

        // Break if no collision was found
        if (!collisionResult.id1 || !collisionResult.id2) {
            break;
        }

        // Roll back and move objects to the collision point
        for (const auto &[verlet]: config.Ecs.GetSystem<Verlet>()) {
            verlet.Revert();
            verlet.Update(collisionResult.tCollision);
        }

        if (config.Ecs.Has<Line>(collisionResult.id2)) {
            auto &verlet1 = config.Ecs.Get<Verlet>(collisionResult.id1);
            auto radius1 = config.Ecs.Get<Circle>(collisionResult.id1).Radius;
            verlet1.Velocity -= Reflect(verlet1.Velocity, config.Ecs.Get<Line>(collisionResult.id2).Normal);
            auto line = config.Ecs.Get<Line>(collisionResult.id2);
            auto point = DistanceLineToPoint(line.Start, line.End, verlet1.Position);
            auto overlapp = radius1 - point.distance;
            if (FloatGreaterThan(overlapp, 0.0f)) {
                verlet1.Position -= line.Normal * overlapp;
            }
        } else {
            auto &verlet1 = config.Ecs.Get<Verlet>(collisionResult.id1);
            auto &verlet2 = config.Ecs.Get<Verlet>(collisionResult.id2);
            RecalculateSphereCollision(verlet1, verlet2);
            auto overlapp = config.Ecs.Get<Circle>(collisionResult.id1).Radius +
                            config.Ecs.Get<Circle>(collisionResult.id2).Radius -
                            Distance(verlet1.Position, verlet2.Position);
            if (FloatGreaterThan(overlapp, 0.0f)) {
                auto normal = NormalBetweenPoints(verlet1.Position, verlet2.Position);
                verlet1.Position += normal * overlapp / 2.0f;
                verlet2.Position -= normal * overlapp / 2.0f;
            }
        }

        t0 += collisionResult.tCollision;
    }
}

