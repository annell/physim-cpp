#include <filesystem>
#include "../thirdParty/ecs/include/EntityComponentSystem.h"
#include <random>
#include "Components.h"
#include "Util.h"
#include "System.h"
#include "Controls.h"

void AddCircle(auto &ecs, auto &worldBoundrarys) {
    auto shape = sf::CircleShape(10);
    auto midX = worldBoundrarys.Position.x + worldBoundrarys.Size.x / 2;
    auto midY = worldBoundrarys.Position.y + worldBoundrarys.Size.y / 2;
    shape.setPosition(midX, midY);
    shape.setFillColor(sf::Color::Cyan);
    auto pos = sf::Vector2f{RandomFloat(20, worldBoundrarys.Size.x - 20), RandomFloat(20, worldBoundrarys.Size.y - 20)};
    shape.setPosition(pos);
    shape.setOrigin(shape.getRadius(), shape.getRadius());
    while (true) {
        bool collision = false;
        for (const auto &[shape2] : ecs.template GetSystem<sf::CircleShape>()) {
            if (Collision(shape, shape2)) {
                pos = sf::Vector2f{RandomFloat(20, worldBoundrarys.Size.x - 20), RandomFloat(20, worldBoundrarys.Size.y - 20)};
                shape.setPosition(pos);
                collision = true;
                break;
            }
        }
        if (!collision) {
            break;
        }
    }
    auto id = ecs.AddEntity();
    ecs.Add(id, id);
    ecs.Add(id, shape);
    ecs.Add(id, Circle{.Radius=ecs.template Get<sf::CircleShape>(id).getRadius()});
    ecs.Add(id, Verlet{pos, {0, 0}, {RandomFloat(-10.1, 10.1), RandomFloat(-10.1, 10.1)}, pos});
}

int main() {
    WorldBoundrarys worldBoundrarys{{0,   0},
                                    {700, 700}};
    sf::RenderWindow sfmlWin(sf::VideoMode(worldBoundrarys.Size.x, worldBoundrarys.Size.y),
                             "Verlet collision simulation");

    bool pause = false;
    bool step = false;
    std::optional<ecs::EntityID> selected;
    ecs::EntityID hoveredId;

    ECS ecs;
    Controls controls;
    controls.RegisterEvent(sf::Event::Closed, [&sfmlWin](auto e) { sfmlWin.close(); });
    controls.RegisterEvent(sf::Event::KeyPressed, [&](auto e) {
        if (e.key.code == sf::Keyboard::Space) {
            pause = !pause;
        } else if (e.key.code == sf::Keyboard::Right) {
            step = true;
        }
    });

    controls.RegisterEvent(sf::Event::MouseButtonPressed, [&](auto e) {
        if (e.mouseButton.button == sf::Mouse::Left) {
            AddCircle(ecs, worldBoundrarys);
        }
        if (e.mouseButton.button == sf::Mouse::Right) {
            if (hoveredId) {
                selected = {hoveredId};
            } else {
                selected = std::nullopt;
            }
        }
    });
    controls.RegisterEvent(sf::Event::MouseMoved, [&](auto e) {
        auto pos = sf::Vector2f{static_cast<float>(e.mouseMove.x), static_cast<float>(e.mouseMove.y)};
        hoveredId = ecs::EntityID();
        for (const auto &[id, shape] : ecs.template GetSystem<ecs::EntityID, sf::CircleShape>()) {
            if (Distance(pos, shape.getPosition()) < shape.getRadius()) {
                hoveredId = id;
                break;
            }
        }
    });
    sf::Font font;
    auto path = std::filesystem::current_path();
    if (!font.loadFromFile(path.generic_string() + "/../../resources/myfont.ttf")) {
        return -1;
    }
    {
        sf::Vector2f A = {0, 0};
        sf::Vector2f B = {worldBoundrarys.Size.x, 0};
        sf::Vector2f C = worldBoundrarys.Size;
        sf::Vector2f D = {0, worldBoundrarys.Size.y};
        auto l1 = ecs.AddEntity();
        ecs.Add(l1, Line{A, B, NormalBetweenPoints(A, B)});
        ecs.Add(l1, l1);
        auto l2 = ecs.AddEntity();
        ecs.Add(l2, Line{B, C, NormalBetweenPoints(B, C)});
        ecs.Add(l2, l2);
        auto l3 = ecs.AddEntity();
        ecs.Add(l3, Line{C, D, NormalBetweenPoints(C, D)});
        ecs.Add(l3, l3);
        auto l4 = ecs.AddEntity();
        ecs.Add(l4, Line{D, A, NormalBetweenPoints(D, A)});
        ecs.Add(l4, l4);
    }
    sf::Text fpsText;
    fpsText.setFont(font);
    fpsText.setPosition(10, 10);

    sf::Text nrPoints;
    nrPoints.setFont(font);
    nrPoints.setPosition(10, 50);

    sf::Clock clock;
    float timer = 0;
    auto fps = std::to_string(1);
    while (sfmlWin.isOpen()) {
        controls.HandleEvents(sfmlWin);
        if (step) {
            pause = false;
        }
        float dt = clock.restart().asSeconds();
        timer += dt;
        if (pause) {
            fps = "Paused";
        } else {
            if (timer > 0.05f) {
                if (ecs.Size() < 20) {
                    AddCircle(ecs, worldBoundrarys);
                }
                timer = 0;
                fps = std::to_string(1 / dt);
            }
            GravitySystem::Run(GravitySystem::Config{
                    .Ecs=ecs,
                    .dt=dt
            });
            CollisionSystem::Run(CollisionSystem::Config{
                    .Ecs=ecs,
                    .worldBoundrarys=worldBoundrarys,
                    .dt=dt
            });
        }
        RenderSystem::Run(RenderSystem::Config{
                .FpsText=fps,
                .Window=sfmlWin,
                .fpsText=fpsText,
                .nrPoints=nrPoints,
                .Ecs=ecs,
                .worldBoundrarys=worldBoundrarys,
                .hoveredId=selected.value_or(hoveredId)
        });
        if (step) {
            pause = true;
            step = false;
        }
    }
    return 0;
}
