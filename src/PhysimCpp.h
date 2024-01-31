//
// Created by Stefan Annell on 2024-01-16.
//

#pragma once
#include <ecs-cpp/EcsCpp.h>
#include "Components.h"
#include "System.h"
#include "Util.h"
#include <iostream>

template <typename TEcs>
class PhysimCpp {
public:
    constexpr PhysimCpp(TEcs& ecs, const WorldBoundrarys& worldBoundrarys)
    : ecs(ecs)
    , worldBoundrarys(worldBoundrarys) {
    }

    void Run(float dt) {
        UpdateVelocity(dt);
        UpdateQuery();

        if constexpr (!ecs::HasTypes<TEcs, Circle, Verlet>()) {
            return;
        }
        const float dtPart = dt / nrIterations;
        for (int i = 0; i < nrIterations; i++) {
            for (const auto [circle1, verlet1, id1, octreeQuery]: ecs.template GetSystem<Circle, Verlet, ecs::EntityID, octreeQuery>()) {
                CircleCircleCollision(circle1, verlet1, id1, octreeQuery, dtPart);
                if constexpr (ecs::HasTypes<TEcs, Line>()) {
                    LineCircleCollision(verlet1, circle1);
                }
            }
        }
    }

private:
    void CircleCircleCollision(auto& circle, auto& verlet, auto& id, auto& octreeQuery, float dt) {
        verlet.PreviousPosition = verlet.Position;
        verlet.Update(dt);
        sf::Vector2f avgDirection;
        sf::Vector2f avgVelocity;
        bool collision = false;
        for (const auto &testPoint: octreeQuery) {
            const auto &id2 = testPoint.Data;
            if (id == id2) {
                continue;
            }

            auto [verlet2, circle2] = ecs.template GetSeveral<Verlet, Circle>(id2);
            auto overlapp = Overlapp(verlet.Position, verlet2.Position, circle.Radius, circle2.Radius);
            if (!overlapp) {
                continue;
            }
            collision = true;
            auto newVelocity = UpdateCircleVelocity(verlet, verlet2);
            avgVelocity += newVelocity;
            verlet2.Velocity += sf::getNormalized(newVelocity) * sf::getLength(verlet2.Velocity) * -1.0f;
            if (overlapp) {
                auto direction = verlet.Position - verlet2.Position;
                avgDirection += normalize(direction) * *overlapp * 0.5f;
            }
        }
        if (collision) {
            verlet.Position += avgDirection;
            auto length = sf::getLength(verlet.Velocity);
            verlet.Velocity += sf::getNormalized(avgVelocity) * length;
        }
    }

    void LineCircleCollision(auto& verlet, auto& circle) {
        for (const auto& [line]: ecs.template GetSystem<Line>()) {
            if (!IntersectMovingCircleLine(circle.Radius, verlet, line)) {
                continue;
            }
            if (auto overlapp = Overlapp(line, verlet.Position, circle.Radius)) {
                verlet.Position -= line.Normal * *overlapp * 1.0f;
            }
            verlet.Velocity -= sf::reflect(verlet.Velocity, line.Normal) * verlet.Bounciness;
        }
    }

    void UpdateVelocity(float dt) {
        if constexpr (!ecs::HasTypes<TEcs, Verlet>()) {
            return;
        }
        for (const auto &[verlet]: ecs.template GetSystem<Verlet>()) {
            verlet.Velocity += verlet.Acceleration * dt;
            verlet.Acceleration = {0, 0};
        }
    }

    void UpdateQuery() {
        if constexpr (!ecs::HasTypes<TEcs, Verlet, octreeQuery, Circle>()) {
            return;
        }
        auto start = std::chrono::high_resolution_clock::now();
        static bool first = true;
        static int n = 0;
        static std::future<void> future;
        n++;
        if (first || is_ready(future)) {
            n = 0;
            future = std::async(std::launch::async, [&](){
                auto octree = MakeOctree(ecs, worldBoundrarys);
                std::vector<std::future<void>> futures;
                int maxParts = 2;
                std::mutex mutex;
                for (int i = 0; i < maxParts; i++) {
                    futures.push_back(std::async(std::launch::async, [&, system=ecs.template GetSystemPart<Circle, Verlet, octreeQuery>(i, maxParts)]() {
                        for (auto [circle, verlet, octreeQuery]: system) {
                            auto queryResults = octree.Query(
                                    Octree::Circle{{verlet.Position.x, verlet.Position.y}, circle.Radius + queryRadius});
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
    TEcs& ecs;
    const WorldBoundrarys worldBoundrarys;
};
