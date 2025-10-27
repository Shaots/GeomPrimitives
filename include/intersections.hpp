#pragma once
#include "geometry.hpp"
#include <cmath>
#include <optional>

namespace geometry::intersections {

/*
 * Класс для поиска пересечений между двумя фигурами
 *
 * Требуется организовать возможность нахождения пересечений только для следующих комбинаций фигур:
 *    - Line   & Line
 *    - Circle & Circle
 *
 * Для всех остальных требуется вернуть std::nullopt
 */
class IntersectionVisitor {
public:
    /* ваш код здесь */
};

inline std::vector<Point2D> GetIntersectPoint(const Shape &shape1, const Shape &shape2) {
    auto get_type_name = [](const auto &shape) -> std::string {
        using T = std::decay_t<decltype(shape)>;
        if constexpr (std::is_same_v<T, Line>)
            return "Line";
        else if constexpr (std::is_same_v<T, Triangle>)
            return "Triangle";
        else if constexpr (std::is_same_v<T, Rectangle>)
            return "Rectangle";
        else if constexpr (std::is_same_v<T, RegularPolygon>)
            return "RegularPolygon";
        else if constexpr (std::is_same_v<T, Circle>)
            return "Circle";
        else if constexpr (std::is_same_v<T, Polygon>)
            return "Polygon";
        else
            return "Unknown";
    };

    auto check = [&](auto &&s1, auto &&s2) -> std::vector<Point2D> {
        using T1 = std::decay_t<decltype(s1)>;
        using T2 = std::decay_t<decltype(s2)>;

        // Line Line
        if constexpr (std::is_same_v<T1, Line> && std::is_same_v<T2, Line>) {
            double det = (s1.end - s1.start).Cross(s2.end - s2.start);
            if (std::fabs(det) < geometry::eps) {
                return {};
            }
            double t = (s2.start - s1.start).Cross(s2.end - s2.start) / det;
            double u = -(s1.end - s1.start).Cross(s2.start - s1.start) / det;
            if (0.0 <= t && t <= 1.0 && 0.0 <= u && u <= 1.0) {
                return std::vector{s1.start * (1 - t) + s1.end * t};
            }
            return {};
        } else if constexpr (std::is_same_v<T1, Circle> && std::is_same_v<T2, Circle>) {
            double r1 = s1.radius;
            double r2 = s2.radius;
            const Point2D &c1 = s1.center_p;
            const Point2D &c2 = s2.center_p;

            double d = c1.DistanceTo(c2);
            double dx = c2.x - c1.x;
            double dy = c2.y - c1.y;
            if (d < eps || d - std::fabs(r1 - r2) < -eps || d - (r1 + r2) > eps) {
                return {};
            }

            if (-eps < d - (r1 + r2) && d - (r1 + r2) < eps) {
                return std::vector{c1 + (c2 - c1) * (r1 / d)};
            }

            double a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
            double h = std::sqrt(r1 * r1 - a * a);

            Point2D p0 = Point2D{c1.x + (dx * a) / d, c1.y + (dy * a) / d};

            return std::vector{Point2D{p0.x + (dy * h) / d, p0.y - (dx * h) / d},
                               Point2D{p0.x - (dy * h) / d, p0.y + (dx * h) / d}};
        } else {
            throw std::logic_error(std::string("Intersection is not supported for ") + get_type_name(s1) + " and " +
                                   get_type_name(s2));
        }
        return {};
    };
    return std::visit(check, shape1, shape2);
}

}  // namespace geometry::intersections