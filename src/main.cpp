#include "SFML/Graphics.hpp"
#include <filesystem>
#include "../thirdParty/ecs/include/EntityComponentSystem.h"
#include "../thirdParty/octree-cpp/src/OctreeCpp.h"
#include <random>

float RandomFloat(float min, float max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min, max);
    return dis(gen);
}

sf::Color RandomColor() {
    return sf::Color(RandomFloat(0, 255), RandomFloat(0, 255), RandomFloat(0, 255));
}

struct Circle {
    float Radius = 10.0;
    float Mass = 1.0;
    float Bounciness = 0.5;
    float Friction = 0.5;
};

struct Verlet {
    sf::Vector2f Position;
    sf::Vector2f Acceleration;
    sf::Vector2f Velocity;
    sf::Vector2f PreviousPosition;
    float MaxVelocity = 1.0;

    void CapVelocity() {
        if (Velocity.x > MaxVelocity) {
            Velocity.x = MaxVelocity;
        }
        if (Velocity.x < -MaxVelocity) {
            Velocity.x = -MaxVelocity;
        }
        if (Velocity.y > MaxVelocity) {
            Velocity.y = MaxVelocity;
        }
        if (Velocity.y < -MaxVelocity) {
            Velocity.y = -MaxVelocity;
        }
    }

    void Update(float dt) {
        PreviousPosition = Position;
        Velocity += Acceleration * dt * dt;
        CapVelocity();
        Position += Velocity;
        Acceleration = {0, 0};
    }
};

float Distance(sf::Vector2f point1, sf::Vector2f point2) {
    float dx = point2.x - point1.x;
    float dy = point2.y - point1.y;
    return std::sqrt(dx * dx + dy * dy);
}

float Dot(sf::Vector2f v1, sf::Vector2f v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

sf::Vector2f Normalize(const sf::Vector2f& vector) {
    float length = std::sqrt(vector.x * vector.x + vector.y * vector.y);
    if (length != 0) {
        return vector / length;
    }
    return vector;
}

struct LineIntersectionResult {
    sf::Vector2f Normal;
    sf::Vector2f Intersection;
    float overlap = 0.0f;
};

std::vector<LineIntersectionResult> CircleLineIntersection(const sf::Vector2f& lineStart, const sf::Vector2f& lineEnd, const sf::Vector2f& circleCenter, float circleRadius) {
    std::vector<LineIntersectionResult> intersections;
    sf::Vector2f lineDirection = lineEnd - lineStart;
    sf::Vector2f lineNormal = {lineDirection.y, -lineDirection.x};
    lineNormal /= std::sqrt(lineNormal.x * lineNormal.x + lineNormal.y * lineNormal.y);
    lineNormal = Normalize(lineNormal);

    float a = lineDirection.x * lineDirection.x + lineDirection.y * lineDirection.y;
    float b = 2 * (lineDirection.x * (lineStart.x - circleCenter.x) + lineDirection.y * (lineStart.y - circleCenter.y));
    float c = circleCenter.x * circleCenter.x + circleCenter.y * circleCenter.y + lineStart.x * lineStart.x + lineStart.y * lineStart.y - 2 * (circleCenter.x * lineStart.x + circleCenter.y * lineStart.y) - circleRadius * circleRadius;
    float discriminant = b * b - 4 * a * c;
    if (discriminant >= 0) {
        float sqrtDiscriminant = std::sqrt(discriminant);
        float t1 = (-b + sqrtDiscriminant) / (2 * a);
        float t2 = (-b - sqrtDiscriminant) / (2 * a);
        if ((t1 >= 0 && t1 <= 1) && (t2 >= 0 && t2 <= 1)) {
            sf::Vector2f intersection = lineStart + lineDirection * (t1 + t2) / 2.0f;
            auto overlap = circleRadius - Distance(circleCenter, intersection);
            intersections.push_back({lineNormal, intersection, overlap});
        }
        else if (t1 >= 0 && t1 <= 1) {
            intersections.push_back({lineNormal, lineStart + lineDirection * t1, 0.0f});
        }
        else if (t2 >= 0 && t2 <= 1) {
            intersections.push_back({lineNormal, lineStart + lineDirection * t2, 0.0f});
        }
    }
    return intersections;
}

void Collision(const LineIntersectionResult& intersection, Verlet& verlet) {
    auto u = Dot(verlet.Velocity, intersection.Normal) * intersection.Normal;
    auto w = verlet.Velocity - u;
    auto d = 0.8f;
    verlet.Velocity = d * w - d * u;
    verlet.Position = verlet.PreviousPosition;
}

bool IsStuck(const Verlet& circle) {
    return circle.Position == circle.PreviousPosition;
}

void Yank(const LineIntersectionResult& intersection, Verlet& verlet) {
    verlet.Position += (verlet.Position - intersection.Intersection) * intersection.overlap * 0.1f;
}

bool HandleCircleBoxCollision(Verlet& circle, const sf::FloatRect& box, float circleRadius) {
    auto intersections = CircleLineIntersection(box.getPosition(), {box.left + box.width, box.top}, circle.Position, circleRadius);
    if (intersections.empty())
        intersections = CircleLineIntersection({box.left + box.width, box.top}, {box.left + box.width, box.top + box.height}, circle.Position, circleRadius);
    if (intersections.empty())
        intersections = CircleLineIntersection({box.left + box.width, box.top + box.height}, {box.left, box.top + box.height}, circle.Position, circleRadius);
    if (intersections.empty())
        intersections = CircleLineIntersection({box.left, box.top + box.height}, box.getPosition(), circle.Position, circleRadius);
    if (intersections.empty())
        return false;
    auto intersection = intersections[0];
    if (IsStuck(circle)) {
        Yank(intersection, circle);
    }
    else {
        Collision(intersection, circle);
    }
    return true;
}

bool HandleCircleCircleCollision(Verlet& circle1, Verlet& circle2, float circleRadius, float circleRadius2) {
    auto distance = Distance(circle1.Position, circle2.Position);
    float overlap = distance - (circleRadius + circleRadius2);
    if (overlap > 0)
        return false;
    auto normal1 = Normalize(circle1.Position - circle2.Position);
    auto normal2 = Normalize(circle2.Position - circle1.Position);
    LineIntersectionResult intersection1 = {normal1, circle1.Position + normal1 * circleRadius, -overlap };
    LineIntersectionResult intersection2 = {normal2, circle2.Position + normal2 * circleRadius2, -overlap };
    if (IsStuck(circle1)) {
        Yank(intersection1, circle1);
        Yank(intersection2, circle2);
    }
    else {
        Collision(intersection1, circle1);
        Collision(intersection2, circle2);
    }
    return true;
}

struct vec {
    float x, y, z = 0;
    auto operator<=>(const vec& rhs) const = default;
};

struct WorldBoundrarys {
    sf::Vector2f Position;
    sf::Vector2f Size;

    [[nodiscard]] sf::FloatRect GetBox() const {
        return {Position, Size};
    }
};

using Octree = OctreeCpp<vec, ecs::EntityID>;

int main() {
        WorldBoundrarys worldBoundrarys{{0, 0}, {700, 700}};
        sf::RenderWindow sfmlWin(sf::VideoMode(worldBoundrarys.Size.x, worldBoundrarys.Size.y), "Verlet collision simulation");
        ecs::ECSManager<sf::CircleShape, Circle, Verlet, ecs::EntityID> ecs;
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
        auto addCircle = [&] () {
            auto id = ecs.AddEntity();
            ecs.Add(id, id);
            ecs.Add(id, sf::CircleShape(RandomFloat(3, 12)));
            ecs.Add(id, Circle{.Radius=ecs.Get<sf::CircleShape>(id).getRadius()});
            auto& shape = ecs.Get<sf::CircleShape>(id);
            auto midX = worldBoundrarys.Position.x + worldBoundrarys.Size.x / 2;
            auto midY = worldBoundrarys.Position.y + worldBoundrarys.Size.y / 2;
            shape.setPosition(midX, midY);
            shape.setFillColor(RandomColor());
            shape.setOrigin(shape.getRadius(), shape.getRadius());
            auto pos = sf::Vector2f{RandomFloat(20, worldBoundrarys.Size.x - 20), RandomFloat(20, worldBoundrarys.Size.y - 20)};
            ecs.Add(id, Verlet{pos, {0, 0}, {RandomFloat(-0.1, 0.1), RandomFloat(-0.1, 0.1)}, pos});
        };

        auto CollisionSystem = [&] () {
            for (const auto& [circle, verlet] : ecs.GetSystem<Circle, Verlet>()) {
                HandleCircleBoxCollision(verlet, worldBoundrarys.GetBox(), circle.Radius);
            }
            Octree octree({{0, 0, 0}, {worldBoundrarys.Size.x, worldBoundrarys.Size.y, 0}});

            auto worldBox = worldBoundrarys.GetBox();
            for (auto& it : ecs) {
                auto verlet = ecs.Get<Verlet>(it.id);
                if (worldBox.contains(verlet.Position)) {
                    octree.Add({{verlet.Position.x, verlet.Position.y}, it.id});
                }
            }

            for (const auto& [circle, verlet, id] : ecs.GetSystem<Circle, Verlet, ecs::EntityID>()) {
                auto query = octree.Query(Octree::Sphere{{verlet.Position.x, verlet.Position.y}, circle.Radius + 12});
                for (const auto& testPoint : query) {
                    if (id != testPoint.Data) {
                        HandleCircleCircleCollision(verlet, ecs.Get<Verlet>(testPoint.Data), circle.Radius, ecs.Get<Circle>(testPoint.Data).Radius);
                    }
                }
            }
        };

        auto ApplyForces = [&] (float dt) {
            for (const auto& [verlet] : ecs.GetSystem<Verlet>()) {
                verlet.Acceleration += {0, 9.81f};
                verlet.Update(dt);
            }
        };

        auto Render = [&] (const std::string& fps) {
            sfmlWin.clear();
            for (const auto& [shape, verlet] : ecs.GetSystem<sf::CircleShape, Verlet>()) {
                shape.setPosition(verlet.Position);
                sfmlWin.draw(shape);
            }
            fpsText.setString(fps);
            nrPoints.setString(std::to_string(ecs.Size()));
            sfmlWin.draw(fpsText);
            sfmlWin.draw(nrPoints);
            sfmlWin.display();
        };

        sf::Clock clock;
        int i = 0;
        float timer = 0;
        auto fps = std::to_string(1);
        while (sfmlWin.isOpen()) {
            auto dt = clock.restart().asSeconds();
            sf::Event e;
            while (sfmlWin.pollEvent(e)) {
                switch (e.type) {
                    case sf::Event::EventType::Closed:
                        sfmlWin.close();
                        break;
                }
            }
            timer += dt;
            if (timer > 0.08f) {
                if (i < 100) {
                    addCircle();
                    i++;
                }
                timer = 0;
                fps = std::to_string(1/dt);
            }

            ApplyForces(dt);
            CollisionSystem();
            Render(fps);
        }
    return 0;
}
