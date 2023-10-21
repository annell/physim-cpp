//
// Created by Stefan Annell on 2023-01-01.
//

#include "../Physics.h"
#include "Util.h"
#include <gtest/gtest.h>
#include "SFML/System.hpp"

TEST(UtilTests, Projection1) {
    sf::Vector2f A = {0, 0};
    sf::Vector2f B = {1, 0};
    auto C = Projection(A, B);
    ASSERT_FLOAT_EQ(C.x, 0.0);
    ASSERT_FLOAT_EQ(C.y, 0.0);
}

TEST(UtilTests, Projection2) {
    sf::Vector2f A = {1, 0.5};
    sf::Vector2f B = {0.2, 1.0};
    auto C = Projection(A, B);
    ASSERT_FLOAT_EQ(C.x, 0.134615391);
    ASSERT_FLOAT_EQ(C.y, 0.673076927);
}

TEST(UtilTests, Projection3) {
    sf::Vector2f A = {1, 0.5};
    sf::Vector2f B = {0.0, 0.0};
    auto C = Projection(A, B);
    ASSERT_FLOAT_EQ(C.x, 0.0f);
    ASSERT_FLOAT_EQ(C.y, 0.0f);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
