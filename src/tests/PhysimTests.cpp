//
// Created by Stefan Annell on 2023-01-01.
//

#include "../Physics.h"
#include "Util.h"
#include <gtest/gtest.h>
#include "SFML/System.hpp"

auto addCircle(ECS &ecs, float radius, const sf::Vector2f &Position, const sf::Vector2f &Velocity) {
    auto shape = sf::CircleShape(radius);
    shape.setOrigin(shape.getRadius(), shape.getRadius());
    shape.setPosition(Position);
    auto id = ecs.AddEntity();
    ecs.Add(id, shape);
    ecs.Add(id, Circle{.Radius=shape.getRadius()});
    ecs.Add(id, Verlet{Position, {0, 0}, Velocity, Position});
    return id;
}

auto buildEmptyECS() {
    ECS ecs;
    WorldBoundrarys worldBoundrarys{{0,   0},
                                    {700, 700}};
    return ecs;
}

auto buildECS() {
    auto ecs = buildEmptyECS();
    sf::Vector2f A = {0, 0};
    sf::Vector2f B = {700, 0};

    auto l1 = ecs.AddEntity();
    ecs.Add(l1, Line{A, B, NormalBetweenPoints(A, B)});

    addCircle(ecs, 10, {0.0f, 0.0f}, {0.1f, 0.1f});

    return ecs;
}

TEST(Physim, AddCircle) {
    auto ecs = buildECS();
    addCircle(ecs, 10, {0.0f, 0.0f}, {0.1f, 0.0f});
    ASSERT_EQ(ecs.Size(), 3);
}

TEST(Physim, OverlappingCircles) {
    auto ecs = buildECS();
    auto id = addCircle(ecs, 10, {10.5f, 10.5f}, {-0.1f, 0.0f});
    float dt = 10.0f;
    for (const auto &[verlet]: ecs.GetSystem<Verlet>()) {
        verlet.Update(dt);
    }
    auto octree = MakeOctree(ecs, WorldBoundrarys{{0,   0},
                                                  {700, 700}});
    ASSERT_EQ(ecs.Size(), 3);
    auto results = CircleCollision(ecs, octree.Query(Octree::All{}), ecs.Get<Verlet>(id), ecs.Get<Circle>(id).Radius,
                                   id, dt);
    ASSERT_TRUE(results);
    ASSERT_FLOAT_EQ(results->tCollision, 0.0);
    ASSERT_EQ(results->id1, ecs::EntityID(2));
    ASSERT_EQ(results->id2, ecs::EntityID(1));
}

TEST(Physim, CirclesMovingAway) {
    auto ecs = buildECS();
    auto id = addCircle(ecs, 5, {15.5f, 15.5f}, {-0.1f, 0.0f});
    float dt = 10.0f;
    for (const auto &[verlet]: ecs.GetSystem<Verlet>()) {
        verlet.Update(dt);
    }
    auto octree = MakeOctree(ecs, WorldBoundrarys{{0,   0},
                                                  {700, 700}});
    ASSERT_EQ(ecs.Size(), 3);
    auto results = CircleCollision(ecs, octree.Query(Octree::All{}), ecs.Get<Verlet>(id), ecs.Get<Circle>(id).Radius,
                                   id, dt);
    ASSERT_FALSE(results);
    ASSERT_FLOAT_EQ(results->tCollision, 100.0f);
}

TEST(Physim, CirclesOverlapping) {
    auto ecs = buildECS();
    auto id = addCircle(ecs, 5, {12.5f, 0.0f}, {0.0f, 0.0f});
    float dt = 10.0f;
    for (const auto &[verlet]: ecs.GetSystem<Verlet>()) {
        verlet.Update(dt);
    }
    auto octree = MakeOctree(ecs, WorldBoundrarys{{0,   0},
                                                  {700, 700}});
    auto results = CircleCollision(ecs, octree.Query(Octree::All{}), ecs.Get<Verlet>(id), ecs.Get<Circle>(id).Radius,
                                   id, dt);
    ASSERT_TRUE(results);
    ASSERT_FLOAT_EQ(results->tCollision, 0.0f);
    ASSERT_EQ(results->id1, ecs::EntityID(2));
    ASSERT_EQ(results->id2, ecs::EntityID(1));
}

TEST(Physim, CirclesMovingCloser) {
    auto ecs = buildECS();
    auto id = addCircle(ecs, 5, {16.0f, 0.0f}, {0.0f, 0.0f});
    float dt = 10.0f;
    for (const auto &[verlet]: ecs.GetSystem<Verlet>()) {
        verlet.Update(dt);
    }
    auto octree = MakeOctree(ecs, WorldBoundrarys{{0,   0},
                                                  {700, 700}});
    auto results = CircleCollision(ecs, octree.Query(Octree::All{}), ecs.Get<Verlet>(id), ecs.Get<Circle>(id).Radius,
                                   id, dt);
    ASSERT_FALSE(results);
    //ASSERT_FLOAT_EQ(results.tCollision, 0.0f);
    //ASSERT_EQ(results.id1, ecs::EntityID(2));
    //ASSERT_EQ(results.id2, ecs::EntityID(1));
}

TEST(Physim, IntersectionLineToPoint1) {
    sf::Vector2f point = {0, 0};
    sf::Vector2f l1 = {0, 0};
    sf::Vector2f l2 = {1, 1};

    auto result = DistanceLineToPoint(l1, l2, point);
    ASSERT_FLOAT_EQ(result.distance, 0.0f);
    ASSERT_FLOAT_EQ(result.t, 0.0f);
}

TEST(Physim, IntersectionLineToPoint2) {
    sf::Vector2f point = {1, 1};
    sf::Vector2f l1 = {0, 0};
    sf::Vector2f l2 = {1, 1};

    auto result = DistanceLineToPoint(l1, l2, point);
    ASSERT_FLOAT_EQ(result.distance, 0.0f);
    ASSERT_FLOAT_EQ(result.t, 1.0f);
}

TEST(Physim, IntersectionLineToPoint3) {
    sf::Vector2f point = {2, 1};
    sf::Vector2f l1 = {0, 0};
    sf::Vector2f l2 = {1, 1};

    auto result = DistanceLineToPoint(l1, l2, point);
    ASSERT_FLOAT_EQ(result.distance, 1.0f);
    ASSERT_FLOAT_EQ(result.t, 1.0f);
}

TEST(Physim, IntersectionLineToPoint4) {
    sf::Vector2f point = {0, -1};
    sf::Vector2f l1 = {0, 0};
    sf::Vector2f l2 = {1, 1};

    auto result = DistanceLineToPoint(l1, l2, point);
    ASSERT_FLOAT_EQ(result.distance, 1.0f);
    ASSERT_FLOAT_EQ(result.t, 0.0f);
}

TEST(Physim, IntersectionLineToPoint5) {
    sf::Vector2f point = {-1, 0};
    sf::Vector2f l1 = {0, 0};
    sf::Vector2f l2 = {1, 1};

    auto result = DistanceLineToPoint(l1, l2, point);
    ASSERT_FLOAT_EQ(result.distance, 1.0f);
    ASSERT_FLOAT_EQ(result.t, 0.0f);
}

TEST(Physim, IntersectionLineToPoint6) {
    sf::Vector2f point = {-1, -1};
    sf::Vector2f l1 = {0, 0};
    sf::Vector2f l2 = {1, 1};

    auto result = DistanceLineToPoint(l1, l2, point);
    ASSERT_FLOAT_EQ(result.distance, 1.4142135f);
    ASSERT_FLOAT_EQ(result.t, 0.0f);
}

TEST(Physim, IntersectionLineToPoint7) {
    sf::Vector2f point = {.5f, .5f};
    sf::Vector2f l1 = {0, 0};
    sf::Vector2f l2 = {1, 1};

    auto result = DistanceLineToPoint(l1, l2, point);
    ASSERT_FLOAT_EQ(result.distance, 0.0f);
    ASSERT_FLOAT_EQ(result.t, 0.5f);
}

TEST(Physim, IntersectionLineToPoint8) {
    sf::Vector2f point = {.2f, .6f};
    sf::Vector2f l1 = {0, 0};
    sf::Vector2f l2 = {1, 1};

    auto result = DistanceLineToPoint(l1, l2, point);
    ASSERT_FLOAT_EQ(result.distance, 0.28284273f);
    ASSERT_FLOAT_EQ(result.t, 0.4f);
}

TEST(Physim, SphereSphereSweep1) {
    sf::Vector2f a0 = {0, 0};
    sf::Vector2f a1 = {1, 1};
    sf::Vector2f b0 = {0, 0};
    sf::Vector2f b1 = {1, 1};
    float ra = 1;
    float rb = 1;
    float u0 = 0;
    auto results = CircleCircleSweep(ra, a0, a1, rb, b0, b1);
    ASSERT_TRUE(results);
    ASSERT_FLOAT_EQ(*results, 0.0f);
}

TEST(Physim, SphereSphereSweep2) {
    sf::Vector2f a0 = {5, 5};
    sf::Vector2f a1 = {5, 5};
    sf::Vector2f b0 = {0, 0};
    sf::Vector2f b1 = {5.01, 5.01};
    float ra = 1;
    float rb = 1;
    float u0 = 0;
    auto results = CircleCircleSweep(ra, a0, a1, rb, b0, b1);
    ASSERT_TRUE(results);
    ASSERT_FLOAT_EQ(*results, 0.99800396f);
}

TEST(Physim, SphereSphereSweep3) {
    sf::Vector2f a0 = {5, 5};
    sf::Vector2f a1 = {0.01f, 0.01f};
    sf::Vector2f b0 = {0, 0};
    sf::Vector2f b1 = {5.01, 5.01};
    float ra = 1;
    float rb = 1;
    float u0 = 0;
    auto results = CircleCircleSweep(ra, a0, a1, rb, b0, b1);
    ASSERT_TRUE(results);
    ASSERT_FLOAT_EQ(*results, 0.5f);
}

TEST(Physim, SphereSphereSweep4) {
    sf::Vector2f a0 = {5, 5};
    sf::Vector2f a1 = {4.01f, 3.01f};
    sf::Vector2f b0 = {0, 0};
    sf::Vector2f b1 = {5.01, 5.01};
    float ra = 1;
    float rb = 1;
    float u0 = 0;
    auto results = CircleCircleSweep(ra, a0, a1, rb, b0, b1);
    ASSERT_TRUE(results);
    ASSERT_FLOAT_EQ(*results, 0.7647059f);
}

TEST(Physim, SphereSphereSweep5) {
    sf::Vector2f a0 = {5, 5};
    sf::Vector2f a1 = {4.01f, 3.01f};
    sf::Vector2f b0 = {0, 0};
    sf::Vector2f b1 = {25.01, 25.01};
    float ra = 1;
    float rb = 1;
    float u0 = 0;
    auto results = CircleCircleSweep(ra, a0, a1, rb, b0, b1);
    ASSERT_TRUE(results);
    ASSERT_FLOAT_EQ(*results, 0.18861209f);
}

TEST(Physim, RecalculateSphereCollision1) {
    Verlet v1 = {.Position={0, 0}, .Velocity={0, 1}};
    Verlet v2 = {.Position={0, 2}, .Velocity={0, -1}};
    RecalculateCircleCollision(v1, v2);
    ASSERT_FLOAT_EQ(v1.Velocity.x, 0.0f);
    ASSERT_FLOAT_EQ(v1.Velocity.y, -1.0f);
    ASSERT_FLOAT_EQ(v2.Velocity.x, 0.0f);
    ASSERT_FLOAT_EQ(v2.Velocity.y, 1.0f);
}

TEST(Physim, RecalculateSphereCollision2) {
    Verlet v1 = {.Position={2, 0}, .Velocity={0, 0}};
    Verlet v2 = {.Position={0, 2}, .Velocity={0, 0}};
    RecalculateCircleCollision(v1, v2);
    ASSERT_FLOAT_EQ(v1.Velocity.x, 0.0f);
    ASSERT_FLOAT_EQ(v1.Velocity.y, 0.0f);
    ASSERT_FLOAT_EQ(v2.Velocity.x, 0.0f);
    ASSERT_FLOAT_EQ(v2.Velocity.y, 0.0f);
}

TEST(Physim, RecalculateSphereCollision3) {
    Verlet v1 = {.Position={2, 0}, .Velocity={-0.5f, 0.5f}};
    Verlet v2 = {.Position={0, 2}, .Velocity={0.5f, -0.5f}};
    RecalculateCircleCollision(v1, v2);
    ASSERT_FLOAT_EQ(v1.Velocity.x, 0.5f);
    ASSERT_FLOAT_EQ(v1.Velocity.y, -0.5f);
    ASSERT_FLOAT_EQ(v2.Velocity.x, -0.5f);
    ASSERT_FLOAT_EQ(v2.Velocity.y, 0.5f);
}

TEST(Physim, RecalculateSphereCollision4) {
    Verlet v1 = {.Position={0, 0}, .Velocity={0, 1}};
    Verlet v2 = {.Position={0, 2}, .Velocity={0, 0}};
    RecalculateCircleCollision(v1, v2);
    ASSERT_FLOAT_EQ(v1.Velocity.x, 0.0f);
    ASSERT_FLOAT_EQ(v1.Velocity.y, 0.0f);
    ASSERT_FLOAT_EQ(v2.Velocity.x, 0.0f);
    ASSERT_FLOAT_EQ(v2.Velocity.y, 1.0f);
}

TEST(Physim, RecalculateSphereCollision5) {
    Verlet v1 = {.Position={0, 0}, .Velocity={0, 1}};
    Verlet v2 = {.Position={0, 2}, .Velocity={0, -1}};
    RecalculateCircleCollision(v1, v2);
    ASSERT_FLOAT_EQ(v1.Velocity.x, 0.0f);
    ASSERT_FLOAT_EQ(v1.Velocity.y, -1.0f);
    ASSERT_FLOAT_EQ(v2.Velocity.x, 0.0f);
    ASSERT_FLOAT_EQ(v2.Velocity.y, 1.0f);
    ASSERT_FLOAT_EQ(Length(v1.Velocity), 1.0f);
    ASSERT_FLOAT_EQ(Length(v2.Velocity), 1.0f);
}

TEST(Physim, RecalculateSphereCollision6) {
    Verlet v1 = {.Position={0, 0}, .Velocity={0, 1}};
    Verlet v2 = {.Position={1, 2}, .Velocity={0, -1}};
    ASSERT_FLOAT_EQ(Length(v1.Velocity), 1.0f);
    ASSERT_FLOAT_EQ(Length(v2.Velocity), 1.0f);
    RecalculateCircleCollision(v1, v2);
    ASSERT_FLOAT_EQ(v1.Velocity.x, -0.8f);
    ASSERT_FLOAT_EQ(v1.Velocity.y, -0.6f);
    ASSERT_FLOAT_EQ(v2.Velocity.x, 0.8f);
    ASSERT_FLOAT_EQ(v2.Velocity.y, 0.6f);
    ASSERT_FLOAT_EQ(Length(v1.Velocity), Length(v2.Velocity));
    ASSERT_FLOAT_EQ(Length(v1.Velocity), 1.0f);
    ASSERT_FLOAT_EQ(Length(v2.Velocity), 1.0f);
}

TEST(Physim, RecalculateSphereCollision7) {
    Verlet v1 = {.Position={0, 0}, .Velocity={0, 1}, .Mass{0.5f}};
    Verlet v2 = {.Position={0, 2}, .Velocity={0, 0}, .Mass{1.0f}};
    RecalculateCircleCollision(v1, v2);
    ASSERT_FLOAT_EQ(v1.Velocity.x, 0.0f);
    ASSERT_FLOAT_EQ(v1.Velocity.y, -0.33333333f);
    ASSERT_FLOAT_EQ(v2.Velocity.x, 0.0f);
    ASSERT_FLOAT_EQ(v2.Velocity.y, 0.666666666f);
    ASSERT_FLOAT_EQ(Length(v2.Velocity) + Length(v1.Velocity), 1.0f);
}

TEST(Physim, SphereCollisionError) {
/*
[11] = {Verlet}
Position = {sf::Vector2f}
x = {float} 215.112549
y = {float} 243.190567
Acceleration = {sf::Vector2f}
x = {float} 0
y = {float} 0
Velocity = {sf::Vector2f}
x = {float} 2.28628826
y = {float} 44.0129967
PreviousPosition = {sf::Vector2f}
x = {float} 215.112549
y = {float} 243.190491
Mass = {float} 1

[11] = {Circle}
Radius = {float} 5.11676407
Mass = {float} 1
Bounciness = {float} 0.5
Friction = {float} 0.5


[18] = {Verlet}
Position = {sf::Vector2f}
x = {float} 205.489822
y = {float} 252.383286
Acceleration = {sf::Vector2f}
x = {float} 0
y = {float} 0
Velocity = {sf::Vector2f}
x = {float} 1.23868752
y = {float} 53.9452934
PreviousPosition = {sf::Vector2f}
x = {float} 205.489822
y = {float} 252.383194
Mass = {float} 1

[18] =
Radius = 8.19516944
Mass = 1
Bounciness = 0.5
Friction = 0.5

dt = 0.000561999972
 */
    {
        // Previous pos
        auto ecs = buildEmptyECS();
        auto id = addCircle(ecs, 5.11676407f, {215.112549f, 243.190491f}, {2.28628826f, 44.0129967f});
        auto id2 = addCircle(ecs, 8.19516944f, {205.489822f, 252.383194f}, {1.23868752f, 53.9452934f});
        float dt = 0.000561999972f;
        for (const auto &[verlet]: ecs.GetSystem<Verlet>()) {
            verlet.Update(dt);
        }
        auto octree = MakeOctree(ecs, WorldBoundrarys{{0,   0},
                                                      {700, 700}});
        ASSERT_EQ(ecs.Size(), 2);
        auto results = CircleCollision(ecs, octree.Query(Octree::All{}), ecs.Get<Verlet>(id),
                                       ecs.Get<Circle>(id).Radius, id, dt);
        ASSERT_TRUE(results);
        ASSERT_FLOAT_EQ(results->tCollision, 0.0);
        ASSERT_EQ(results->id1, ecs::EntityID(1));
        ASSERT_EQ(results->id2, ecs::EntityID(2));

    }
    {
        // Current pos
        auto ecs = buildEmptyECS();
        auto id = addCircle(ecs, 5.11676407f, {215.112549f, 243.190567f}, {2.28628826f, 44.0129967f});
        auto id2 = addCircle(ecs, 8.19516944f, {205.489822f, 252.383286f}, {1.23868752f, 53.9452934f});
        float dt = 0.000561999972f;
        for (const auto &[verlet]: ecs.GetSystem<Verlet>()) {
            verlet.Update(dt);
        }
        auto octree = MakeOctree(ecs, WorldBoundrarys{{0,   0},
                                                      {700, 700}});
        ASSERT_EQ(ecs.Size(), 2);
        auto results = CircleCollision(ecs, octree.Query(Octree::All{}), ecs.Get<Verlet>(id),
                                       ecs.Get<Circle>(id).Radius, id, dt);
        ASSERT_TRUE(results);
        ASSERT_FLOAT_EQ(results->tCollision, 0.0);
        ASSERT_EQ(results->id1, ecs::EntityID(1));
        ASSERT_EQ(results->id2, ecs::EntityID(2));

    }
}

TEST(Physim, SphereCollisionError2) {
/*

 [12] =
 Position =
  x = 518.16626
  y = 649.816956
 Acceleration =
  x = 0
  y = 0
 Velocity =
  x = 8.36222267
  y = 54.8974876
 PreviousPosition =
  x = 518.166077
  y = 649.815552
 PreviousVelocity =
  x = 8.36222267
  y = 54.8974876
 PreviousDt = 0.0000252880855
 Mass = 1

 [12] =
 Radius = 3.8132751
 Mass = 1
 Bounciness = 0.5
 Friction = 0.5


 [16] =
 Position =
  x = 529.01825
  y = 655.19397
 Acceleration =
  x = 0
  y = 0
 Velocity =
  x = 9.70443725
  y = 54.2360229
 PreviousPosition =
  x = 529.018005
  y = 655.192626
 PreviousVelocity =
  x = -20.8121605
  y = 26.3217239
 PreviousDt = 0.0000252880855
 Mass = 1

 [16] =
 Radius = 8.31039237
 Mass = 1
 Bounciness = 0.5
 Friction = 0.5

 0.00109599996
 */
    {
        // Previous pos
        auto ecs = buildEmptyECS();
        auto id = addCircle(ecs, 3.8132751f, {518.166077f, 649.815552f}, {8.36222267f, 54.8974876f});
        auto id2 = addCircle(ecs, 8.31039237f, {529.018005f, 655.192626f}, {-20.8121605f, 26.3217239f});
        float dt = 0.0000252880855f;
        for (const auto &[verlet]: ecs.GetSystem<Verlet>()) {
            verlet.Update(dt);
        }
        auto octree = MakeOctree(ecs, WorldBoundrarys{{0,   0},
                                                      {700, 700}});
        ASSERT_EQ(ecs.Size(), 2);
        auto results = CircleCollision(ecs, octree.Query(Octree::All{}), ecs.Get<Verlet>(id),
                                       ecs.Get<Circle>(id).Radius, id, dt);
        ASSERT_TRUE(results);
        ASSERT_FLOAT_EQ(results->tCollision, 0.0);
        ASSERT_EQ(results->id1, ecs::EntityID(1));
        ASSERT_EQ(results->id2, ecs::EntityID(2));

    }
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
