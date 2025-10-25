#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <intersections.hpp>
#include <optional>
#include <variant>

namespace geometry::queries {

template <class... Ts>
struct Multilambda : Ts... {
    using Ts::operator()...;
};

/*
 * Класс для поиска расстояния от точки до фигуры
 *
 * Требуется организовать возможность нахождения расстояния для всех возможных фигур типа-суммы Shape
 */
struct PointToShapeDistanceVisitor {
    Point2D point;

    explicit PointToShapeDistanceVisitor(const Point2D &p) : point(p) {}

    double operator()(const Line &line) const {
        const Point2D &a = line.start;
        const Point2D &b = line.end;
        Point2D ab = b - a;
        Point2D ap = point - a;
        double t = ap.Dot(ab) / ab.Dot(ab);
        t = std::clamp(t, 0.0, 1.0);
        Point2D closest = a + ab * t;

        return point.DistanceTo(closest);
    }

    double operator()(const Triangle &triangle) const {
        Line edges[3] = {{triangle.a, triangle.b}, {triangle.b, triangle.c}, {triangle.c, triangle.a}};

        double min_distance = std::numeric_limits<double>::max();
        for (const auto &edge : edges) {
            double dist = (*this)(edge);
            if (dist < min_distance) {
                min_distance = dist;
            }
        }
        return min_distance;
    }

    double operator()(const Rectangle &rect) const {
        double closest_x = std::clamp(point.x, rect.bottom_left.x, rect.bottom_left.x + rect.width);
        double closest_y = std::clamp(point.y, rect.bottom_left.y, rect.bottom_left.y + rect.height);

        if (closest_x == point.x && closest_y == point.y) {
            return 0.0;
        }

        return point.DistanceTo(Point2D{closest_x, closest_y});
    }

    double operator()(const RegularPolygon &poly) const {
        auto vertices = poly.Vertices();
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < vertices.size(); ++i) {
            Line edge{vertices[i], vertices[(i + 1) % vertices.size()]};
            double dist = (*this)(edge);
            if (dist < min_distance) {
                min_distance = dist;
            }
        }
        return min_distance;
    }

    double operator()(const Circle &circle) const {
        double distance_to_center = point.DistanceTo(circle.center_p);
        return std::max(0.0, distance_to_center - circle.radius);
    }

    double operator()(const Polygon &poly) const {
        auto vertices = poly.Vertices();
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < vertices.size(); ++i) {
            Line edge{vertices[i], vertices[(i + 1) % vertices.size()]};
            double dist = (*this)(edge);
            if (dist < min_distance) {
                min_distance = dist;
            }
        }
        return min_distance;
    }
};

/*
 * Класс для поиска расстояния между двумя фигурами
 *
 * Требуется организовать возможность нахождения расстояния только для следующих комбинаций фигур:
 *    - Any    & Point
 *    - Line   & Line
 *    - Circle & Circle
 *
 * Важно: вы можете выбрать любой метод нахождения расстояния, даже если он даёт не точный результат
 *
 * Для всех остальных требуется вернуть пустое значение
 */
struct ShapeToShapeDistanceVisitor {
    std::optional<double> operator()(const Shape &shape, const Point2D &point) const {
        PointToShapeDistanceVisitor visitor{point};
        return std::visit(visitor, shape);
    }

    std::optional<double> operator()(const Line &line1, const Line &line2) const {
        auto intersect = geometry::intersections::GetIntersectPoint(line1, line2);
        if (!intersect.empty()) {
            return 0.0;
        }

        // Otherwise find minimum distance between endpoints and the other line
        PointToShapeDistanceVisitor visitor1{line1.start};
        PointToShapeDistanceVisitor visitor2{line1.end};
        PointToShapeDistanceVisitor visitor3{line2.start};
        PointToShapeDistanceVisitor visitor4{line2.end};

        double dist1 = visitor1(line2);
        double dist2 = visitor2(line2);
        double dist3 = visitor3(line1);
        double dist4 = visitor4(line1);

        return std::min({dist1, dist2, dist3, dist4});
    }

    std::optional<double> operator()(const Circle &circle1, const Circle &circle2) const {
        double distance_between_centers = circle1.center_p.DistanceTo(circle2.center_p);
        double distance = std::max(0.0, distance_between_centers - circle1.radius - circle2.radius);
        return distance;
    }

    // Default case for unsupported combinations
    template <typename T1, typename T2>
    std::optional<double> operator()(const T1 &, const T2 &) const {
        return std::nullopt;
    }
};

/*
 * Функции-помощники
 */
inline double DistanceToPoint(const Shape &shape, const Point2D &point) {
    PointToShapeDistanceVisitor visitor{point};
    return std::visit(visitor, shape);
}

inline BoundingBox GetBoundBox(const Shape &shape) {
    return std::visit([](const auto &s) { return s.BoundBox(); }, shape);
}

inline double GetHeight(const Shape &shape) {
    return std::visit([](const auto &s) { return s.Height(); }, shape);
}

inline bool BoundingBoxesOverlap(const Shape &shape1, const Shape &shape2) {
    BoundingBox box1 = GetBoundBox(shape1);
    BoundingBox box2 = GetBoundBox(shape2);

    return !(box1.max_x < box2.min_x || box1.min_x > box2.max_x || box1.max_y < box2.min_y || box1.min_y > box2.max_y);
}

std::optional<double> DistanceBetweenShapes(const Shape &shape1, const Shape &shape2) {
    ShapeToShapeDistanceVisitor visitor;
    return std::visit(visitor, shape1, shape2);
}

}  // namespace geometry::queries