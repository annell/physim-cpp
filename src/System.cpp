//
// Created by Stefan Annell on 2023-07-25.
//

#include "System.h"
#include "Util.h"
#include "../thirdParty/octree-cpp/src/OctreeCpp.h"
#include "Physics.h"

void RenderSystem::Run(Config &config) {
    config.Window.clear();
    for (const auto &[shape, verlet, id]: config.Ecs.GetSystem<sf::CircleShape, Verlet, ecs::EntityID>()) {
        shape.setPosition(verlet.Position);
        config.Window.draw(shape);
        if (!config.worldBoundrarys.GetBox().contains(verlet.Position)) {
            config.Ecs.RemoveEntity(id);
        }
    }
    config.fpsText.setString(config.FpsText);
    config.nrPoints.setString(std::to_string(config.Ecs.Size()));
    config.Window.draw(config.fpsText);
    config.Window.draw(config.nrPoints);
    config.Window.display();
}

void GravitySystem::Run(Config &config) {
    for (const auto &[verlet]: config.Ecs.GetSystem<Verlet>()) {
        verlet.Acceleration += {0, 9.81f};
        verlet.Update(config.dt);
    }
}

using Octree = OctreeCpp<vec, ecs::EntityID>;

void CollisionSystem::Run(Config &config) {
    Octree octree({{0, 0, 0},
                   {config.worldBoundrarys.Size.x, config.worldBoundrarys.Size.y, 0}});

    for (auto &it: config.Ecs) {
        auto verlet = config.Ecs.Get<Verlet>(it.id);
        if (config.worldBoundrarys.GetBox().contains(verlet.Position)) {
            octree.Add({{verlet.Position.x, verlet.Position.y}, it.id});
        }
    }

    for (const auto &[circle, verlet, id]: config.Ecs.GetSystem<Circle, Verlet, ecs::EntityID>()) {
        auto query = octree.Query(Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + 12});
        for (const auto &testPoint: query) {
            if (id != testPoint.Data) {
                HandleCircleCircleCollision(verlet, config.Ecs.Get<Verlet>(testPoint.Data), circle.Radius,
                                            config.Ecs.Get<Circle>(testPoint.Data).Radius);
            }
        }
    }
}

void BoundraryCollisionSystem::Run(Config &config) {
    for (const auto &[circle, verlet]: config.Ecs.GetSystem<Circle, Verlet>()) {
        HandleCircleBoxCollision(verlet, config.worldBoundrarys.GetBox(), circle.Radius);
    }
}
