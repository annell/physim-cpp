#include <filesystem>
#include "../thirdParty/ecs/include/EntityComponentSystem.h"
#include <random>
#include "Components.h"
#include "Util.h"
#include "System.h"

void AddCircle(auto &ecs, auto &worldBoundrarys) {
    auto id = ecs.AddEntity();
    ecs.Add(id, id);
    ecs.Add(id, sf::CircleShape(RandomFloat(3, 12)));
    ecs.Add(id, Circle{.Radius=ecs.template Get<sf::CircleShape>(id).getRadius()});
    auto &shape = ecs.template Get<sf::CircleShape>(id);
    auto midX = worldBoundrarys.Position.x + worldBoundrarys.Size.x / 2;
    auto midY = worldBoundrarys.Position.y + worldBoundrarys.Size.y / 2;
    shape.setPosition(midX, midY);
    shape.setFillColor(RandomColor());
    shape.setOrigin(shape.getRadius(), shape.getRadius());
    auto pos = sf::Vector2f{RandomFloat(20, worldBoundrarys.Size.x - 20), RandomFloat(20, worldBoundrarys.Size.y - 20)};
    ecs.Add(id, Verlet{pos, {0, 0}, {RandomFloat(-0.1, 0.1), RandomFloat(-0.1, 0.1)}, pos});
}

void HandleEvents(sf::RenderWindow &sfmlWin) {
    sf::Event e;
    while (sfmlWin.pollEvent(e)) {
        switch (e.type) {
            case sf::Event::EventType::Closed:
                sfmlWin.close();
                break;
        }
    }
}

int main() {
    WorldBoundrarys worldBoundrarys{{0,   0},
                                    {700, 700}};
    sf::RenderWindow sfmlWin(sf::VideoMode(worldBoundrarys.Size.x, worldBoundrarys.Size.y),
                             "Verlet collision simulation");
    ECS ecs;
    sf::Font font;
    auto path = std::filesystem::current_path();
    if (!font.loadFromFile(path.generic_string() + "/../resources/myfont.ttf")) {
        return -1;
    }
    sf::Text fpsText;
    fpsText.setFont(font);
    fpsText.setPosition(10, 10);

    sf::Text nrPoints;
    nrPoints.setFont(font);
    nrPoints.setPosition(10, 50);

    sf::Clock clock;
    int i = 0;
    float timer = 0;
    auto fps = std::to_string(1);
    while (sfmlWin.isOpen()) {
        HandleEvents(sfmlWin);
        float dt = clock.restart().asSeconds();
        timer += dt;
        if (timer > 0.08f) {
            if (i < 100) {
                AddCircle(ecs, worldBoundrarys);
                i++;
            }
            timer = 0;
            fps = std::to_string(1 / dt);
        }
        {
            GravitySystem::Config config{
                    .Ecs=ecs,
                    .dt=dt
            };
            GravitySystem::Run(config);
        }

        {
            CollisionSystem::Config config{
                    .Ecs=ecs,
                    .worldBoundrarys=worldBoundrarys
            };
            CollisionSystem::Run(config);
        }

        {
            RenderSystem::Config config{
                    .FpsText=fps,
                    .Window=sfmlWin,
                    .fpsText=fpsText,
                    .nrPoints=nrPoints,
                    .Ecs=ecs
            };

            RenderSystem::Run(config);
        }
    }
    return 0;
}
