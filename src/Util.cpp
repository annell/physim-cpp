#include "Util.h"

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

float Distance(const sf::Vector2f &point1, const sf::Vector2f &point2) {
    float dx = point2.x - point1.x;
    float dy = point2.y - point1.y;
    return std::sqrt(dx * dx + dy * dy);
}

float Dot(const sf::Vector2f &v1, const sf::Vector2f &v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

sf::Vector2f Normalize(const sf::Vector2f &vector) {
    float length = std::sqrt(vector.x * vector.x + vector.y * vector.y);
    if (length != 0) {
        return vector / length;
    }
    return vector;
}
