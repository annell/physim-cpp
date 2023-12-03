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
#include <chrono>

constexpr float PI_VertexPerCircle() {
    return M_PI / vertexPerCircle;
}

void addCircle(sf::VertexArray &array, sf::Vector2f position, float radius, sf::Color color) {
    for (int i = 0; i < vertexPerCircle; i += 1) {
        sf::Vertex v0;
        v0.position = sf::Vector2f(position.x, position.y);
        v0.color = color;
        array.append(v0);

        sf::Vertex v1;
        float angle = i * 2 * PI_VertexPerCircle();
        v1.position = sf::Vector2f(position.x + cos(angle) * radius, position.y + sin(angle) * radius);
        v1.color = color;
        array.append(v1);

        sf::Vertex v2;
        angle = (i + 1) * 2 * PI_VertexPerCircle();
        v2.position = sf::Vector2f(position.x + cos(angle) * radius, position.y + sin(angle) * radius);
        v2.color = color;
        array.append(v2);
    }
}

void RenderSystem::Run(const Config &config) {
    config.Window.clear();
    std::vector<ecs::EntityID> entitiesToRemove;
    std::optional<sf::Vector2f> hoveredPos = config.hoveredId ? config.Ecs.Get<Verlet>(config.hoveredId).Position
                                                              : std::optional<sf::Vector2f>();
    sf::VertexArray points(sf::Triangles);
    points.resize(config.Ecs.Size() * vertexPerCircle);

    for (const auto &[circle, verlet, id, octreeSwitch]: config.Ecs.GetSystem<Circle, Verlet, ecs::EntityID, OctreeSwitch>()) {
        auto radius = circle.Radius;
        sf::Color color;
        if (id == config.hoveredId) {
            color = sf::Color::Red;
        } else {
            if (hoveredPos) {
                auto distance = sf::distance(verlet.Position, *hoveredPos);
                if (distance <= radius + queryRadius) {
                    color = sf::Color::Green;
                } else {
                    color = circle.Color;
                }
            } else {
                color = circle.Color;
            }
        }
        addCircle(points, verlet.Position, radius, color);
        if (id == config.hoveredId) {
            sf::CircleShape outline;
            outline.setRadius(radius + queryRadius);
            outline.setOrigin(radius + queryRadius, radius + queryRadius);
            outline.setPosition(verlet.Position);
            outline.setFillColor(sf::Color::Transparent);
            outline.setOutlineColor(sf::Color::Red);
            outline.setOutlineThickness(2);

            config.Window.draw(outline);
            std::optional<float> minDistance;
            std::optional<float> minOverlapp;
            for (const auto &testPoint: octreeSwitch.GetUsableQuery()) {
                const auto &id2 = testPoint.Data;
                if (id != id2) {
                    auto &circle2 = config.Ecs.Get<Circle>(id2);
                    auto &verlet2 = config.Ecs.Get<Verlet>(id2);
                    // Draw line between the two points
                    sf::Vertex line[] = {
                            sf::Vertex(verlet.Position),
                            sf::Vertex(verlet2.Position)
                    };
                    config.Window.draw(line, 2, sf::Lines);
                    minDistance = std::min(minOverlapp.value_or(1000.0f), sf::distance(verlet.Position, verlet2.Position));
                    minOverlapp = std::min(minOverlapp.value_or(1000.0f),
                                           sf::distance(verlet.Position, verlet2.Position) - (radius + circle2.Radius));
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

    for (const auto& line : config.lines) {
        sf::Vertex lineShape[] = {
                sf::Vertex(line.Start),
                sf::Vertex(line.End)
        };
        config.Window.draw(lineShape, 2, sf::Lines);
    }

    config.fpsText.setString(config.FpsText);
    config.nrPoints.setString(std::to_string(config.Ecs.Size()));
    config.Window.draw(points);
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

std::optional<std::future<void>> UpdateQuery(const DiscreteCollisionSystem::Config &config) {
    static bool first = true;
    if (first) {
        auto octree = MakeOctree(config.Ecs, config.worldBoundrarys);
        for (const auto &[circle, verlet, octreeQuery]: config.Ecs.GetSystem<Circle, Verlet, OctreeSwitch>()) {
            octreeQuery.Query1 = octree.Query(
                    Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + queryRadius});
        }
        first = false;
        return std::nullopt;
    }

    auto a = std::async(std::launch::async, [&]() {
        auto octree = MakeOctree(config.Ecs, config.worldBoundrarys);
        for (const auto &[circle, verlet, octreeQuery]: config.Ecs.GetSystem<Circle, Verlet, OctreeSwitch>()) {
            auto& query = octreeQuery.GetUpdatingQuery();
            query = octree.Query(
                    Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + queryRadius});
            octreeQuery.Switch();
        }
    });
    return a;
}

void DiscreteCollisionSystem::Run(const Config &config) {
    static std::optional<std::future<void>> a;
    if (!a) {
        a = UpdateQuery(config);
    } else {
        a->wait();
        a = UpdateQuery(config);
    }
    float dtPart = config.dt / nrIterations;
    for (int i = 0; i < nrIterations; i++) {
        for (const auto [circle1, verlet1, id1, octreeSwitch]: config.Ecs.GetSystem<Circle, Verlet, ecs::EntityID, OctreeSwitch>()) {
            verlet1.PreviousPosition = verlet1.Position;
            verlet1.Update(dtPart);
            sf::Vector2f avgDirection;
            sf::Vector2f avgVelocity;
            bool collision = false;
            for (const auto &testPoint: octreeSwitch.GetUsableQuery()) {
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
                if (!IntersectMovingCircleLine(circle1.Radius, verlet1, line)) {
                    continue;
                }
                auto [verlet1, circle1] = config.Ecs.GetSeveral<Verlet, Circle>(id1);
                if (auto overlapp = Overlapp(line, verlet1.Position, circle1.Radius)) {
                    verlet1.Position -= line.Normal * *overlapp * 1.0f;
                }
                verlet1.Velocity -= sf::reflect(verlet1.Velocity, line.Normal) * verlet1.Bounciness;
            }
        }
    }
}
