//
// Created by Stefan Annell on 2023-07-25.
//
#pragma once

#include "Util.h"
#include "Components.h"

struct LineIntersectionResult {
    sf::Vector2f Normal;
    sf::Vector2f Intersection;
    float overlap = 0.0f;
};

std::vector<LineIntersectionResult>
CircleLineIntersection(const sf::Vector2f &lineStart, const sf::Vector2f &lineEnd, const sf::Vector2f &circleCenter,
                       float circleRadius) {
    std::vector<LineIntersectionResult> intersections;
    sf::Vector2f lineDirection = lineEnd - lineStart;
    sf::Vector2f lineNormal = {lineDirection.y, -lineDirection.x};
    lineNormal /= std::sqrt(lineNormal.x * lineNormal.x + lineNormal.y * lineNormal.y);
    lineNormal = Normalize(lineNormal);

    float a = lineDirection.x * lineDirection.x + lineDirection.y * lineDirection.y;
    float b = 2 * (lineDirection.x * (lineStart.x - circleCenter.x) + lineDirection.y * (lineStart.y - circleCenter.y));
    float c = circleCenter.x * circleCenter.x + circleCenter.y * circleCenter.y + lineStart.x * lineStart.x +
              lineStart.y * lineStart.y - 2 * (circleCenter.x * lineStart.x + circleCenter.y * lineStart.y) -
              circleRadius * circleRadius;
    float discriminant = b * b - 4 * a * c;
    if (discriminant >= 0) {
        float sqrtDiscriminant = std::sqrt(discriminant);
        float t1 = (-b + sqrtDiscriminant) / (2 * a);
        float t2 = (-b - sqrtDiscriminant) / (2 * a);
        if ((t1 >= 0 && t1 <= 1) && (t2 >= 0 && t2 <= 1)) {
            sf::Vector2f intersection = lineStart + lineDirection * (t1 + t2) / 2.0f;
            auto overlap = circleRadius - Distance(circleCenter, intersection);
            intersections.push_back({lineNormal, intersection, overlap});
        } else if (t1 >= 0 && t1 <= 1) {
            intersections.push_back({lineNormal, lineStart + lineDirection * t1, 0.0f});
        } else if (t2 >= 0 && t2 <= 1) {
            intersections.push_back({lineNormal, lineStart + lineDirection * t2, 0.0f});
        }
    }
    return intersections;
}

void Collision(const LineIntersectionResult &intersection, Verlet &verlet) {
    auto u = Dot(verlet.Velocity, intersection.Normal) * intersection.Normal;
    auto w = verlet.Velocity - u;
    auto d = 0.8f;
    verlet.Velocity = d * w - d * u;
    verlet.Position = verlet.PreviousPosition;
}

bool IsStuck(const Verlet &circle) {
    return circle.Position == circle.PreviousPosition;
}

void Yank(const LineIntersectionResult &intersection, Verlet &verlet) {
    verlet.Position += (verlet.Position - intersection.Intersection) * intersection.overlap * 0.1f;
}

bool HandleCircleBoxCollision(Verlet &circle, const sf::FloatRect &box, float circleRadius) {
    auto intersections = CircleLineIntersection(box.getPosition(), {box.left + box.width, box.top}, circle.Position,
                                                circleRadius);
    if (intersections.empty())
        intersections = CircleLineIntersection({box.left + box.width, box.top},
                                               {box.left + box.width, box.top + box.height}, circle.Position,
                                               circleRadius);
    if (intersections.empty())
        intersections = CircleLineIntersection({box.left + box.width, box.top + box.height},
                                               {box.left, box.top + box.height}, circle.Position, circleRadius);
    if (intersections.empty())
        intersections = CircleLineIntersection({box.left, box.top + box.height}, box.getPosition(), circle.Position,
                                               circleRadius);
    if (intersections.empty())
        return false;
    auto intersection = intersections[0];
    if (IsStuck(circle)) {
        Yank(intersection, circle);
    } else {
        Collision(intersection, circle);
    }
    return true;
}

bool HandleCircleCircleCollision(Verlet &circle1, Verlet &circle2, float circleRadius, float circleRadius2) {
    auto distance = Distance(circle1.Position, circle2.Position);
    float overlap = distance - (circleRadius + circleRadius2);
    if (overlap > 0)
        return false;
    auto normal1 = Normalize(circle1.Position - circle2.Position);
    auto normal2 = Normalize(circle2.Position - circle1.Position);
    LineIntersectionResult intersection1 = {normal1, circle1.Position + normal1 * circleRadius, -overlap};
    LineIntersectionResult intersection2 = {normal2, circle2.Position + normal2 * circleRadius2, -overlap};
    if (IsStuck(circle1)) {
        Yank(intersection1, circle1);
        Yank(intersection2, circle2);
    } else {
        Collision(intersection1, circle1);
        Collision(intersection2, circle2);
    }
    return true;
}


