#include <filesystem>
#include <ecs-cpp/EcsCpp.h>
#include <random>
#include "Components.h"
#include "Util.h"
#include "System.h"
#include "Physics.h"
#include "Controls.h"
#include "PhysimCpp.h"
#include <SFMLMath.hpp>

void AddCircle(auto &ecs, auto &worldBoundrarys) {
    auto pos = sf::Vector2f{RandomFloat(20, worldBoundrarys.Size.x - 20), RandomFloat(20, worldBoundrarys.Size.y - 20)};
    while (true) {
        bool collision = false;
        for (const auto &[circle, verlet]: ecs.template GetSystem<Circle, Verlet>()) {
            if (Overlapp(verlet.Position, pos, circle.Radius, circleRadius)) {
                pos = sf::Vector2f{RandomFloat(20, worldBoundrarys.Size.x - 20),
                                   RandomFloat(20, worldBoundrarys.Size.y - 20)};
                collision = true;
                break;
            }
        }
        if (!collision) {
            break;
        }
    }
    ecs.BuildEntity(
            Circle{.Radius=circleRadius, .Color=RandomColor()},
            Verlet{pos, {0, 0}, {RandomFloat(-10.1, 10.1), RandomFloat(-10.1, 10.1)}, pos},
            octreeQuery{}
    );
}

void AddBoundingBox(auto &ecs, auto &worldBoundrarys) {
    sf::Vector2f A = {0, 0};
    sf::Vector2f B = {worldBoundrarys.Size.x, 0};
    sf::Vector2f C = worldBoundrarys.Size;
    sf::Vector2f D = {0, worldBoundrarys.Size.y};
    ecs.BuildEntity(Line{A, B, sf::normalBetweenPoints(A, B)});
    ecs.BuildEntity(Line{B, C, sf::normalBetweenPoints(B, C)});
    ecs.BuildEntity(Line{C, D, sf::normalBetweenPoints(C, D)});
    ecs.BuildEntity(Line{D, A, sf::normalBetweenPoints(D, A)});
}

int main() {
    WorldBoundrarys worldBoundrarys{{0,   0},
                                    {1200, 700}};
    sf::RenderWindow sfmlWin(sf::VideoMode(worldBoundrarys.Size.x, worldBoundrarys.Size.y),
                             "Verlet collision simulation");

    bool pause = false;
    bool step = false;
    std::optional<ecs::EntityID> selected;
    ecs::EntityID hoveredId;

    ECS ecs;
    PhysimCpp physimCpp(ecs, worldBoundrarys);
    Controls controls;
    controls.RegisterEvent(sf::Event::Closed, [&sfmlWin](auto) { sfmlWin.close(); });
    controls.RegisterEvent(sf::Event::KeyPressed, [&](auto e) {
        if (e.key.code == sf::Keyboard::Space) {
            pause = !pause;
        } else if (e.key.code == sf::Keyboard::Right) {
            step = true;
        }
    });

    Line newLine;
    Lines lines;

    controls.RegisterEvent(sf::Event::MouseButtonPressed, [&](auto e) {
        if (e.mouseButton.button == sf::Mouse::Left) {
            newLine.Start = sf::Vector2f{static_cast<float>(e.mouseButton.x), static_cast<float>(e.mouseButton.y)};
        }
        if (e.mouseButton.button == sf::Mouse::Right) {
            if (hoveredId) {
                selected = {hoveredId};
            } else {
                selected = std::nullopt;
            }
        }
    });

    controls.RegisterEvent(sf::Event::MouseButtonReleased, [&](auto e) {
        if (e.mouseButton.button == sf::Mouse::Left) {
            newLine.End = sf::Vector2f{static_cast<float>(e.mouseButton.x), static_cast<float>(e.mouseButton.y)};
            newLine.Normal = sf::normalBetweenPoints(newLine.Start, newLine.End);
            ecs.BuildEntity(Line{newLine});
        }
    });
    controls.RegisterEvent(sf::Event::MouseMoved, [&](auto e) {
        auto pos = sf::Vector2f{static_cast<float>(e.mouseMove.x), static_cast<float>(e.mouseMove.y)};
        hoveredId = ecs::EntityID();
        for (const auto &[id, circle, verlet]: ecs.template GetSystem<ecs::EntityID, Circle, Verlet>()) {
            if (sf::distance(pos, verlet.Position) < circle.Radius) {
                hoveredId = id;
                break;
            }
        }
    });
    sf::Font font;
    auto path = std::filesystem::current_path();
    if (!font.loadFromFile(path.generic_string() + "/myfont.ttf")) {
        return -1;
    }
    AddBoundingBox(ecs, worldBoundrarys);

    sf::Text fpsText;
    fpsText.setFont(font);
    fpsText.setPosition(10, 10);

    sf::Text nrPoints;
    nrPoints.setFont(font);
    nrPoints.setPosition(10, 50);

    sf::Clock clock;
    auto fps = std::to_string(1);
    bool doneAddingCircles = false;
    while (sfmlWin.isOpen()) {
        float dt = clock.restart().asSeconds();
        controls.HandleEvents(sfmlWin);
        if (step) {
            pause = false;
            dt = 1/60.0f;
        }
        if (!doneAddingCircles){
            for (int i = 0; i < 50 && ecs.Size() < nrCircles; i++) {
                AddCircle(ecs, worldBoundrarys);
                pause = true;
            }
            if (ecs.Size() == nrCircles) {
                doneAddingCircles = true;
            }
        }
        if (pause) {
            fps = "Paused";
        } else {
            if (pause) {
                fps = "Done adding circles";
            } else {
                fps = std::to_string(1 / dt);
                GravitySystem::Run(GravitySystem::Config{
                        .Ecs=ecs,
                        .dt=dt
                });
                physimCpp.Run(dt);
            }
        }
        RenderSystem::Run(RenderSystem::Config{
                .FpsText=fps,
                .Window=sfmlWin,
                .fpsText=fpsText,
                .nrPoints=nrPoints,
                .Ecs=ecs,
                .worldBoundrarys=worldBoundrarys,
                .hoveredId=selected.value_or(hoveredId),
                .lines=lines
        });
        if (step) {
            pause = true;
            step = false;
        }
    }
    return 0;
}
