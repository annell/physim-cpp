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
    for (const auto &[shape, verlet, id, query]: config.Ecs.GetSystem<sf::CircleShape, Verlet, ecs::EntityID, octreeQuery>()) {
        shape.setPosition(verlet.Position);
        if (id == config.hoveredId) {
            shape.setFillColor(sf::Color::Red);
        } else {
            if (hoveredPos) {
                auto distance = sf::distance(verlet.Position, *hoveredPos);
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

std::optional<CollisionResult> CheckCollisions(const ContinousCollisionSystem::Config &config, const auto& octree, float tLeft) {
    std::optional<CollisionResult> collision;
    for (const auto &[circle, verlet, id]: config.Ecs.GetSystem<Circle, Verlet, ecs::EntityID>()) {
        auto query = octree.Query(
                Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + queryRadius});
        if (auto circleResults = FirstCircleCollision(config.Ecs, query, verlet, circle.Radius, id, tLeft)) {
            if (!collision) {
                collision = circleResults;
                continue;
            }
            collision = Min(circleResults.value(), collision.value());
        }
        if (auto lineResults = FirstLineCollision(config.Ecs, verlet, circle.Radius, tLeft)) {
            lineResults->id1 = id;
            if (!collision) {
                collision = lineResults;
                continue;
            }
            collision = Min(lineResults.value(), collision.value());
        }
    }
    return collision;
}

void StepForward(auto system, float tLeft) {
    for (const auto &[verlet]: system) {
        verlet.PreviousPosition = verlet.Position;
        verlet.Update(tLeft);
    }
}

void RollBack(auto system, float dt) {
    for (const auto &[verlet]: system) {
        verlet.Revert();
        verlet.Update(dt);
    }
}

void ResolveLineCollision(ECS& ecs, const CollisionResult& collisionResult) {
    auto [verlet1, circle1] = ecs.GetSeveral<Verlet, Circle>(collisionResult.id1);
    auto line = ecs.Get<Line>(collisionResult.id2);
    verlet1.Velocity -= sf::reflect(verlet1.Velocity, line.Normal) * verlet1.Bounciness;
    auto overlapp = Overlapp(line, verlet1.Position, circle1.Radius);
    while (overlapp) {
        auto NewPos = verlet1.Position - line.Normal * (*overlapp + 1.0f);
        verlet1.Position = NewPos;
        overlapp = Overlapp(line, verlet1.Position, circle1.Radius);
    }
}

void ResolveCircleCollision(ECS& ecs, const CollisionResult& collisionResult) {
    auto [verlet1, circle1] = ecs.GetSeveral<Verlet, Circle>(collisionResult.id1);
    auto [verlet2, circle2] = ecs.GetSeveral<Verlet, Circle>(collisionResult.id2);
    UpdateCircleVelocity(verlet1, verlet2);
    auto overlapp = Overlapp(verlet1.Position, verlet2.Position, circle1.Radius, circle2.Radius);
    while (overlapp) {
        auto normal = sf::normalBetweenPoints(verlet1.Position, verlet2.Position);
        verlet1.Position += normal * (*overlapp + 1.0f);
        verlet2.Position -= normal * (*overlapp + 1.0f);
        overlapp = Overlapp(verlet1.Position, verlet2.Position, circle1.Radius, circle2.Radius);
    }
}

void ContinousCollisionSystem::Run(const Config &config) {
    auto octree = MakeOctree(config.Ecs, config.worldBoundrarys);

    float t0 = 0.0f;
    if (config.dt > 1) {
        std::cout << "dt is to big" << std::endl;
        return;
    }

    while (t0 < config.dt) {
        auto tLeft = config.dt - t0;
        StepForward(config.Ecs.GetSystem<Verlet>(), tLeft);
        auto collisionResult = CheckCollisions(config, octree, tLeft);

        if (!collisionResult) {
            break;
        }

        //if (collisionResult->tCollision < tLeft) {
            collisionResult->tCollision *= 0.95;
            RollBack(config.Ecs.GetSystem<Verlet>(), collisionResult->tCollision);
        //}

        if (collisionResult->Type == CollisionType::Line) {
            ResolveLineCollision(config.Ecs, *collisionResult);
        } else {
            ResolveCircleCollision(config.Ecs, *collisionResult);
        }

        t0 += collisionResult->tCollision;
    }
}

void ResolveCollisions(const DiscreteCollisionSystem::Config &config, float dt) {
    static int i = 2;
    i++;
    if (i >= 5) {
        auto octree = MakeOctree(config.Ecs, config.worldBoundrarys);
        for (const auto &[circle, verlet, octreeQuery]: config.Ecs.GetSystem<Circle, Verlet, octreeQuery>()) {
            octreeQuery = octree.Query(
                    Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + queryRadius});
        }
        i = 0;
    }
    float dtPart = dt / nrIterations;
    for (int i = 0; i < nrIterations; i++) {
        for (const auto [circle1, verlet1, id1, query]: config.Ecs.GetSystem<Circle, Verlet, ecs::EntityID, octreeQuery>()) {
                verlet1.PreviousPosition = verlet1.Position;
                verlet1.Update(dtPart);
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
                    UpdateCircleVelocity(verlet1, verlet2);
                    if (overlapp) {
                        auto direction = verlet1.Position - verlet2.Position;
                        direction = normalize(direction);
                        verlet1.Position += direction * *overlapp * 0.5f;
                        verlet2.Position -= direction * *overlapp * 0.5f;
                    }
                }
                for (const auto& [line, id2]: config.Ecs.GetSystem<Line, ecs::EntityID>()) {
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
