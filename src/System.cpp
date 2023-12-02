//
// Created by Stefan Annell on 2023-07-25.
//

#include "System.h"
#include "Util.h"
#include "Physics.h"
#include <iostream>
#include <cassert>
#include <SFMLMath.hpp>
#include <future>


void RenderSystem::Run(const Config &config) {
    config.Window.clear();
    std::vector<ecs::EntityID> entitiesToRemove;
    std::optional<sf::Vector2f> hoveredPos = config.hoveredId ? config.Ecs.Get<Verlet>(config.hoveredId).Position
                                                              : std::optional<sf::Vector2f>();
    std::vector<sf::Vertex> points;
    points.reserve(config.Ecs.Size());

    for (const auto &[shape, circle, verlet, id, query]: config.Ecs.GetSystem<sf::CircleShape, Circle, Verlet, ecs::EntityID, octreeQuery>()) {
        sf::Vertex point;
        point.position = verlet.Position;
        //shape.setPosition(verlet.Position);
        if (id == config.hoveredId) {
            //shape.setFillColor(sf::Color::Red);
            point.color = sf::Color::Red;
        } else {
            if (hoveredPos) {
                auto distance = sf::distance(verlet.Position, *hoveredPos);
                if (distance <= shape.getRadius() + queryRadius) {
                    //shape.setFillColor(sf::Color::Green);
                    point.color = sf::Color::Green;
                } else {
                    //shape.setFillColor(circle.Color);
                    point.color = circle.Color;
                }
            } else {
                //shape.setFillColor(circle.Color);
                point.color = circle.Color;
            }
        }
        //config.Window.draw(shape);
        points.push_back(point);
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
                    minDistance = std::min(minOverlapp.value_or(1000.0f), sf::distance(verlet.Position, verlet2.Position));
                    minOverlapp = std::min(minOverlapp.value_or(1000.0f),
                                           sf::distance(verlet.Position, verlet2.Position) - (radius + shape.getRadius()));
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
    config.Window.draw(points.data(), points.size(), sf::Points);

    for (const auto& line : config.lines) {
        sf::Vertex lineShape[] = {
                sf::Vertex(line.Start),
                sf::Vertex(line.End)
        };
        config.Window.draw(lineShape, 2, sf::Lines);
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

void ResolveCollisions(const DiscreteCollisionSystem::Config &config, float dt) {
    static bool first = true;
    if (first) {
        auto octree = MakeOctree(config.Ecs, config.worldBoundrarys);
        for (const auto &[circle, verlet, octreeQuery]: config.Ecs.GetSystem<Circle, Verlet, octreeQuery>()) {
            octreeQuery = octree.Query(
                    Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + queryRadius});
        }
        first = false;
    }
    else {
        static int i = 0;
        if (i > 3) {
            auto a1 = std::async(std::launch::async, [&]() {
                auto octree = MakeOctree(config.Ecs, config.worldBoundrarys);
                for (const auto &[circle, verlet, octreeQuery]: config.Ecs.GetSystem<Circle, Verlet, octreeQuery>()) {
                    octreeQuery = octree.Query(
                            Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + queryRadius});
                }
            });
            i = 0;
        }
        i++;
    }
    float dtPart = dt / nrIterations;
    for (int i = 0; i < nrIterations; i++) {
        for (const auto [circle1, verlet1, id1, query]: config.Ecs.GetSystem<Circle, Verlet, ecs::EntityID, octreeQuery>()) {
                verlet1.PreviousPosition = verlet1.Position;
                verlet1.Update(dtPart);
                sf::Vector2f avgDirection;
                sf::Vector2f avgVelocity;
                bool collision = false;
                for (const auto &testPoint: query) {
                    const auto &id2 = testPoint.Data;
                    if (id1 == id2) {
                        continue;
                    }

                    auto [verlet2, circle2] = config.Ecs.GetSeveral<Verlet, Circle>(id2);
                    auto overlapp = Overlapp(verlet1.Position, verlet2.Position, circle1.Radius, circle2.Radius);
                    if (!overlapp) {
                        continue;
                    }
                    collision = true;
                    auto newVelocity = UpdateCircleVelocity(verlet1, verlet2);
                    avgVelocity += newVelocity;
                    verlet2.Velocity += sf::getNormalized(newVelocity) * sf::getLength(verlet2.Velocity) * -1.0f;
                    if (overlapp) {
                        auto direction = verlet1.Position - verlet2.Position;
                        avgDirection += normalize(direction) * *overlapp * 0.5f;
                    }
                }
                if (collision) {
                    verlet1.Position += avgDirection;
                    auto length = sf::getLength(verlet1.Velocity);
                    verlet1.Velocity += sf::getNormalized(avgVelocity) * length;
                }
                for (const auto& line: config.Lines) {
                    float t = 1.0f;
                    if (!IntersectMovingCircleLine(circle1.Radius, verlet1, line, t)) {
                        continue;
                    }
                    auto [verlet1, circle1] = config.Ecs.GetSeveral<Verlet, Circle>(id1);
                    auto overlapp = Overlapp(line, verlet1.Position, circle1.Radius);
                    if (overlapp) {
                        verlet1.Position -= line.Normal * *overlapp * 1.0f;
                    }
                    verlet1.Velocity -= sf::reflect(verlet1.Velocity, line.Normal) * verlet1.Bounciness;
                }
        }
    }
}

void DiscreteCollisionSystem::Run(const Config &config) {
    ResolveCollisions(config, config.dt);
}
