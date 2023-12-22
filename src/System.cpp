//
// Created by Stefan Annell on 2023-07-25.
//

#include "System.h"
#include "Util.h"
#include "Physics.h"
#include <iostream>
#include <cassert>
#include <SFMLMath.hpp>

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

void addPoint(sf::VertexArray &array, sf::Vector2f position, sf::Color color) {
    sf::Vertex v0;
    v0.position = sf::Vector2f(position.x, position.y);
    v0.color = color;
    array.append(v0);
}

void RenderSystem::Run(const Config &config) {
    config.Window.clear();
    std::vector<ecs::EntityID> entitiesToRemove;
    std::optional<sf::Vector2f> hoveredPos = config.hoveredId ? config.Ecs.Get<Verlet>(config.hoveredId).Position
                                                              : std::optional<sf::Vector2f>();
    sf::VertexArray points(pointRendering ? sf::Points : sf::Triangles);
    points.resize(config.Ecs.Size());

    for (const auto &[circle, verlet, id, octreeQuery]: config.Ecs.GetSystem<Circle, Verlet, ecs::EntityID, octreeQuery>()) {
        sf::Color color;
        if (id == config.hoveredId) {
            color = sf::Color::Red;
        } else {
            if (hoveredPos) {
                auto distance = sf::distance(verlet.Position, *hoveredPos);
                if (distance <= circle.Radius + queryRadius) {
                    color = sf::Color::Green;
                } else {
                    color = circle.Color;
                }
            } else {
                color = circle.Color;
            }
        }
        if (pointRendering) {
            addPoint(points, verlet.Position, color);
        } else {
            addCircle(points, verlet.Position, circle.Radius, color);
        }
        if (id == config.hoveredId) {
            sf::CircleShape outline;
            outline.setRadius(circle.Radius + queryRadius);
            outline.setOrigin(circle.Radius + queryRadius, circle.Radius + queryRadius);
            outline.setPosition(verlet.Position);
            outline.setFillColor(sf::Color::Transparent);
            outline.setOutlineColor(sf::Color::Red);
            outline.setOutlineThickness(2);

            config.Window.draw(outline);
            std::optional<float> minDistance;
            std::optional<float> minOverlapp;
            for (const auto &testPoint: octreeQuery) {
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
                                           sf::distance(verlet.Position, verlet2.Position) - (circle.Radius + circle2.Radius));
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

    for (const auto &[line]: config.Ecs.GetSystem<Line>()) {
        sf::Vertex lineShape[] = {
                sf::Vertex(line.Start),
                sf::Vertex(line.End)
        };
        config.Window.draw(lineShape, 2, sf::Lines);
    }

    /*
    auto octree = MakeOctree(config.Ecs, config.worldBoundrarys);
    for (const auto& boundrary : octree.GetBoundaries()) {
        sf::RectangleShape rectangle;
        auto boundrarySize = boundrary.GetSize();
        rectangle.setSize({boundrarySize.x, boundrarySize.y});
        rectangle.setPosition({boundrary.Min.x, boundrary.Min.y});
        rectangle.setFillColor(sf::Color::Transparent);
        rectangle.setOutlineColor(sf::Color::White);
        rectangle.setOutlineThickness(1);
        config.Window.draw(rectangle);
    }
     */

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

void UpdateQuery(const DiscreteCollisionSystem::Config &config) {
    auto start = std::chrono::high_resolution_clock::now();
    static bool first = true;
    static int n = 0;
    static std::future<void> future;
    n++;
    if (first || is_ready(future)) {
        n = 0;
        future = std::async(std::launch::async, [&](){
            auto octree = MakeOctree(config.Ecs, config.worldBoundrarys);
            std::vector<std::future<void>> futures;
            int maxParts = 2;
            std::mutex mutex;
            for (int i = 0; i < maxParts; i++) {
                futures.push_back(std::async(std::launch::async, [&, system=config.Ecs.GetSystemPart<Circle, Verlet, octreeQuery>(i, maxParts)]() {
                    for (auto [circle, verlet, octreeQuery]: system) {
                        auto queryResults = octree.Query(
                                Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + queryRadius});
                        if (!queryResults.empty()) {
                            std::lock_guard<std::mutex> lock(mutex);
                            octreeQuery = queryResults;
                        }
                    }
                }));
            }
            for (auto& future: futures) {
                future.wait();
            }
        });
    }
    if (first) {
        future.wait();
        first = false;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end-start;
    std::cout << "Time spent waiting: " << diff.count() << " seconds" << std::endl;
    std::cout << "Iterations: " << n << std::endl;
}

void DiscreteCollisionSystem::Run(const Config &config) {

    UpdateQuery(config);

    float dtPart = config.dt / nrIterations;
    for (int i = 0; i < nrIterations; i++) {
        for (const auto [circle1, verlet1, id1, octreeQuery]: config.Ecs.GetSystem<Circle, Verlet, ecs::EntityID, octreeQuery>()) {
            verlet1.PreviousPosition = verlet1.Position;
            verlet1.Update(dtPart);
            sf::Vector2f avgDirection;
            sf::Vector2f avgVelocity;
            bool collision = false;
            for (const auto &testPoint: octreeQuery) {
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
            for (const auto& [line]: config.Ecs.GetSystem<Line>()) {
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
