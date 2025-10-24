#pragma once
#include "geometry.hpp"
#include <algorithm>
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

    /* ваш код здесь */
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
    // ShapeToShapeDistanceVisitor visitor;
    // return std::visit(visitor, shape1, shape2);
    return std::nullopt;
}

}  // namespace geometry::queries