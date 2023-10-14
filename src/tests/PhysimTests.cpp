//
// Created by Stefan Annell on 2023-01-01.
//

#include "../Physics.h"
#include "Util.h"
#include <gtest/gtest.h>
#include "SFML/System.hpp"

TEST(Physim, hello) {
    ASSERT_EQ(1, 1);
    ECS ecs;
    WorldBoundrarys worldBoundrarys{{0,   0},
                                    {700, 700}};
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

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
