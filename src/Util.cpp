#include "Util.h"

#include "SFML/Graphics.hpp"
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

bool Collision(const sf::CircleShape &circle1, const sf::CircleShape &circle2) {
    float distance = Distance(circle1.getPosition(), circle2.getPosition());
    return distance < circle1.getRadius() + circle2.getRadius();
}

float Distance(const sf::Vector2f &point1, const sf::Vector2f &point2) {
    float dx = point2.x - point1.x;
    float dy = point2.y - point1.y;
    return std::sqrt(dx * dx + dy * dy);
}

float Dot(const sf::Vector2f &v1, const sf::Vector2f &v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

sf::Vector2f NormalBetweenPoints(const sf::Vector2f &point1, const sf::Vector2f &point2) {
    sf::Vector2f direction = point2 - point1;
    sf::Vector2f normal = {direction.y, -direction.x};
    normal /= std::sqrt(normal.x * normal.x + normal.y * normal.y);
    return normal;
}

sf::Vector2f Reflect(const sf::Vector2f &vector, const sf::Vector2f &normal) {
    return 2 * Dot(vector, normal) * normal;
}

float Length(const sf::Vector2f &vector) {
    return std::sqrt(vector.x * vector.x + vector.y * vector.y);
}

bool FloatEqual(float a, float b) {
    return fabs(a - b) < std::numeric_limits<float>::epsilon();
}

bool FloatIsZero(float a) {
    return FloatEqual(a, 0.0f);
}

bool FloatLessThan(float a, float b) {
    return a < b and not FloatEqual(a, b);
}

bool FloatGreaterThan(float a, float b) {
    return not FloatLessThan(a, b);
}

sf::Vector2f Normalize(const sf::Vector2f &vector) {
    float length = std::sqrt(vector.x * vector.x + vector.y * vector.y);
    if (length != 0) {
        return vector / length;
    }
    return vector;
}

sf::Vector2f Projection(const sf::Vector2f &vector, const sf::Vector2f &axis) {
    if (axis == sf::Vector2f(0, 0)) {
        return sf::Vector2f(0, 0);
    }
    float k = Dot(vector, axis) / Dot(axis, axis);
    return k * axis;
}

float Hypot2(const sf::Vector2f &v1, const sf::Vector2f &v2) {
    return Dot(v1 - v2, v1 - v2);
}


Octree MakeOctree(ECS& ecs, const WorldBoundrarys& worldBoundrarys) {
    Octree octree({{0,                             0,                             0},
                   {worldBoundrarys.Size.x, worldBoundrarys.Size.y, 0}});

    for (auto &it: ecs) {
        if (ecs.Has<Verlet>(it.id)) {
            auto& verlet = ecs.Get<Verlet>(it.id);
            if (worldBoundrarys.GetBox().contains(verlet.Position)) {
                octree.Add({{verlet.Position.x, verlet.Position.y}, it.id});
            }
        }
    }

    return std::move(octree);
}
IntersectionResult DistanceLineToPoint(const sf::Vector2f& A, const sf::Vector2f& B, const sf::Vector2f& C) {
    auto AC = C - A;
    auto AB = B - A;

    auto D = Projection(AC, AB) + A;
    auto AD = D - A;

    auto k = std::abs(AB.x) > std::abs(AB.y) ? AD.x / AB.x : AD.y / AB.y;

    if (k <= 0) {
        return {std::sqrt(Hypot2(C, A)), 0.0f};
    } else if (k >= 1) {
        return {std::sqrt(Hypot2(C, B)), 1.0f};
    }
    return {std::sqrt(Hypot2(C, D)), std::abs(D.x / (A.x + B.x))};
}

// minimum distance (squared) between vertices, i.e. minimum segment length (squared)
#define EPSILON_MIN_VERTEX_DISTANCE_SQUARED 0.00000001

// An arbitrary tiny epsilon.  If you use float instead of double, you'll probably want to change this to something like 1E-7f
#define EPSILON_TINY 1.0E-14

// Arbitrary general epsilon.  Useful for places where you need more "slop" than EPSILON_TINY (which is most places).
// If you use float instead of double, you'll likely want to change this to something like 1.192092896E-04
#define EPSILON_GENERAL 1.192092896E-07

bool AreValuesEqual(double val1, double val2, double tolerance)
{
    if (val1 >= (val2 - tolerance) && val1 <= (val2 + tolerance))
    {
        return true;
    }

    return false;
}


double PointToPointDistanceSquared(double p1x, double p1y, double p2x, double p2y)
{
    double dx = p2x - p1x;
    double dy = p2y - p1y;
    return (dx * dx) + (dy * dy);
}


double PointSegmentDistanceSquared( double px, double py,
                                    double p1x, double p1y,
                                    double p2x, double p2y,
                                    double& t,
                                    float& qx, float& qy)
{
    double dx = p2x - p1x;
    double dy = p2y - p1y;
    double dp1x = px - p1x;
    double dp1y = py - p1y;
    const double segLenSquared = (dx * dx) + (dy * dy);
    if (AreValuesEqual(segLenSquared, 0.0, EPSILON_MIN_VERTEX_DISTANCE_SQUARED))
    {
        // segment is a point.
        qx = p1x;
        qy = p1y;
        t = 0.0;
        return ((dp1x * dp1x) + (dp1y * dp1y));
    }
    else
    {
        t = ((dp1x * dx) + (dp1y * dy)) / segLenSquared;
        if (t <= EPSILON_TINY)
        {
            // intersects at or to the "left" of first segment vertex (p1x, p1y).  If t is approximately 0.0, then
            // intersection is at p1.  If t is less than that, then there is no intersection (i.e. p is not within
            // the 'bounds' of the segment)
            if (t >= -EPSILON_TINY)
            {
                // intersects at 1st segment vertex
                t = 0.0;
            }
            // set our 'intersection' point to p1.
            qx = p1x;
            qy = p1y;
            // Note: If you wanted the ACTUAL intersection point of where the projected lines would intersect if
            // we were doing PointLineDistanceSquared, then qx would be (p1x + (t * dx)) and qy would be (p1y + (t * dy)).
        }
        else if (t >= (1.0 - EPSILON_TINY))
        {
            // intersects at or to the "right" of second segment vertex (p2x, p2y).  If t is approximately 1.0, then
            // intersection is at p2.  If t is greater than that, then there is no intersection (i.e. p is not within
            // the 'bounds' of the segment)
            if (t <= (1.0 + EPSILON_TINY))
            {
                // intersects at 2nd segment vertex
                t = 1.0;
            }
            qx = p2x;
            qy = p2y;
            // Note: If you wanted the ACTUAL intersection point of where the projected lines would intersect if
            // we were doing PointLineDistanceSquared, then qx would be (p1x + (t * dx)) and qy would be (p1y + (t * dy)).
        }
        else
        {
            // The projection of the point to the point on the segment that is perpendicular succeeded and the point
            // is 'within' the bounds of the segment.  Set the intersection point as that projected point.
            qx = ((1.0 - t) * p1x) + (t * p2x);
            qy = ((1.0 - t) * p1y) + (t * p2y);
            // for debugging
            //ASSERT(AreValuesEqual(qx, p1x + (t * dx), EPSILON_TINY));
            //ASSERT(AreValuesEqual(qy, p1y + (t * dy), EPSILON_TINY));
        }
        // return the squared distance from p to the intersection point.
        double dpqx = px - qx;
        double dpqy = py - qy;
        return ((dpqx * dpqx) + (dpqy * dpqy));
    }
}


double SegmentSegmentDistanceSquared(   double p1x, double p1y,
                                        double p2x, double p2y,
                                        double p3x, double p3y,
                                        double p4x, double p4y,
                                        float& qx, float& qy)
{
    // check to make sure both segments are long enough (i.e. verts are farther apart than minimum allowed vert distance).
    // If 1 or both segments are shorter than this min length, treat them as a single point.
    double segLen12Squared = PointToPointDistanceSquared(p1x, p1y, p2x, p2y);
    double segLen34Squared = PointToPointDistanceSquared(p3x, p3y, p4x, p4y);
    double t = 0.0;
    double minDist2 = 1E+38;
    float tmpQx, tmpQy = 0;
    double tmpD2 = 0;
    if (segLen12Squared <= EPSILON_MIN_VERTEX_DISTANCE_SQUARED)
    {
        qx = p1x;
        qy = p1y;
        if (segLen34Squared <= EPSILON_MIN_VERTEX_DISTANCE_SQUARED)
        {
            // point to point
            minDist2 = PointToPointDistanceSquared(p1x, p1y, p3x, p3y);
        }
        else
        {
            // point - seg
            minDist2 = PointSegmentDistanceSquared(p1x, p1y, p3x, p3y, p4x, p4y, tmpD2, tmpQx, tmpQy);
        }
        return minDist2;
    }
    else if (segLen34Squared <= EPSILON_MIN_VERTEX_DISTANCE_SQUARED)
    {
        // seg - point
        minDist2 = PointSegmentDistanceSquared(p3x, p3y, p1x, p1y, p2x, p2y, t, qx, qy);
        return minDist2;
    }

    // if you have a point class and/or methods to do cross products, you can use those here.
    // This is what we're actually doing:
    // Point2D delta43(p4x - p3x, p4y - p3y);    // dir of p3 -> p4
    // Point2D delta12(p1x - p2x, p1y - p2y);    // dir of p2 -> p1
    // double d = delta12.Cross2D(delta43);
    double d = ((p4y - p3y) * (p1x - p2x)) - ((p1y - p2y) * (p4x - p3x));
    bool bParallel = AreValuesEqual(d, 0.0, EPSILON_GENERAL);

    if (!bParallel)
    {
        // segments are not parallel.  Check for intersection.
        // Point2D delta42(p4x - p2x, p4y - p2y);    // dir of p2 -> p4
        // t = 1.0 - (delta42.Cross2D(delta43) / d);
        t = 1.0 - ((((p4y - p3y) * (p4x - p2x)) - ((p4y - p2y) * (p4x - p3x))) / d);
        double seg12TEps = sqrt(EPSILON_MIN_VERTEX_DISTANCE_SQUARED / segLen12Squared);
        if (t >= -seg12TEps && t <= (1.0 + seg12TEps))
        {
            // inside [p1,p2].   Segments may intersect.
            // double s = 1.0 - (delta12.Cross2D(delta42) / d);
            double s = 1.0 - ((((p4y - p2y) * (p1x - p2x)) - ((p1y - p2y) * (p4x - p2x))) / d);
            double seg34TEps = sqrt(EPSILON_MIN_VERTEX_DISTANCE_SQUARED / segLen34Squared);
            if (s >= -seg34TEps && s <= (1.0 + seg34TEps))
            {
                // segments intersect!
                minDist2 = 0.0;
                qx = ((1.0 - t) * p1x) + (t * p2x);
                qy = ((1.0 - t) * p1y) + (t * p2y);
                // for debugging
                //double qsx = ((1.0 - s) * p3x) + (s * p4x);
                //double qsy = ((1.0 - s) * p3y) + (s * p4y);
                //ASSERT(AreValuesEqual(qx, qsx, EPSILON_MIN_VERTEX_DISTANCE_SQUARED));
                //ASSERT(AreValuesEqual(qy, qsy, EPSILON_MIN_VERTEX_DISTANCE_SQUARED));
                return minDist2;
            }
        }
    }

    // Segments do not intersect.   Find closest point and return dist.   No other way at this
    // point except to just brute-force check each segment end-point vs opposite segment.  The
    // minimum distance of those 4 tests is the closest point.
    minDist2 = PointSegmentDistanceSquared(p3x, p3y, p1x, p1y, p2x, p2y, t, qx, qy);
    tmpD2 = PointSegmentDistanceSquared(p4x, p4y, p1x, p1y, p2x, p2y, t, tmpQx, tmpQy);
    if (tmpD2 < minDist2)
    {
        qx = tmpQx;
        qy = tmpQy;
        minDist2 = tmpD2;
    }
    tmpD2 = PointSegmentDistanceSquared(p1x, p1y, p3x, p3y, p4x, p4y, t, tmpQx, tmpQy);
    if (tmpD2 < minDist2)
    {
        qx = p1x;
        qy = p1y;
        minDist2 = tmpD2;
    }
    tmpD2 = PointSegmentDistanceSquared(p2x, p2y, p3x, p3y, p4x, p4y, t, tmpQx, tmpQy);
    if (tmpD2 < minDist2)
    {
        qx = p2x;
        qy = p2y;
        minDist2 = tmpD2;
    }

    return minDist2;
}

double SegmentSegmentDistance(const sf::Vector2f& L1Start, const sf::Vector2f& L1End, const sf::Vector2f& L2Start, const sf::Vector2f& L2End, sf::Vector2f& Out) {
    return std::sqrt(SegmentSegmentDistanceSquared(L1Start.x, L1Start.y, L1End.x, L1End.y, L2Start.x, L2Start.y, L2End.x, L2End.y, Out.x, Out.y));
}
