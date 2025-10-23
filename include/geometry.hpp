#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <expected>
#include <format>
#include <numbers>
#include <numeric>
#include <optional>
#include <print>
#include <ranges>
#include <variant>
#include <vector>

namespace geometry {

/*
 * Добавьте к методам класса Point2D и Lines2DDyn все необходимые аттрибуты и спецификаторы
 * Важно: Возвращаемый тип и принимаемые аргументы менять не нужно
 */
struct Point2D {
    double x, y;

    constexpr Point2D() : x(0), y(0) {}
    constexpr Point2D(double x, double y) : x(x), y(y) {}

    // Comparison
    bool operator<(const Point2D &other) const { return x < other.x && y < other.y; }
    bool operator==(const Point2D &other) const {
        return std::fabs(x - other.x) < 1e-10 && std::fabs(y - other.y) < 1e-10;
    }

    // Binary math operators
    Point2D operator+(const Point2D &other) const { return {x + other.x, y + other.y}; }
    Point2D operator-(const Point2D &other) const { return {x - other.x, y - other.y}; }
    Point2D operator*(double value) const { return {x * value, y * value}; }
    Point2D operator/(double value) const { return {x / value, y / value}; }

    // Binary geometry operations
    double Dot(const Point2D &other) { return x * other.x + y * other.y; }
    double Cross(const Point2D &other) { return x * other.y - y * other.x; }
    double Length() { return std::sqrt(x * x + y * y); }
    double DistanceTo(const Point2D &other) const { return (*this - other).Length(); }

    Point2D Normalize() {
        const double len = Length();
        return len > 0 ? Point2D{x / len, y / len} : Point2D{0, 0};
    }
};

template <size_t N>
struct Lines2D {
    std::array<double, N> x;
    std::array<double, N> y;
};

struct Lines2DDyn {
    std::vector<double> x;
    std::vector<double> y;

    void Reserve(size_t n) {
        x.reserve(n);
        y.reserve(n);
    }
    void PushBack(Point2D p) {
        x.push_back(p.x);
        y.push_back(p.y);
    }
    void PushBack(double px, double py) {
        x.push_back(px);
        y.push_back(py);
    }
    Point2D Front() { return {x.front(), y.front()}; }
};

struct BoundingBox {
    double min_x, min_y, max_x, max_y;

    bool operator==(const BoundingBox &other) const {
        return min_x == other.min_x && min_y == other.min_y && max_x == other.max_x && max_y == other.max_y;
    }

    void Overlaps();

    double Width() { return max_x - min_x; }

    double Height() { return max_y - min_y; }

    Point2D Center() { return Point2D{min_x + Width() / 2, min_y + Height() / 2}; }
};

struct Line {
    Point2D start, end;

    double length() { return start.DistanceTo(end); }

    Point2D Direction() { return end - start; }

    BoundingBox BoundBox() {
        return BoundingBox{std::min(start.x, end.x), std::min(start.y, end.y), std::max(start.x, end.x),
                           std::max(start.y, end.y)};
    }

    double Height() { return std::fabs(end.y - start.y); }

    Point2D Center() { return (start + end) / 2; }

    std::array<Point2D, 2> Vertices() { return {Point2D{start.x, start.y}, {end.x, end.y}}; }
    Lines2D<2> Lines() const { return {{start.x, end.x}, {start.y, end.y}}; }
};

struct Triangle {
    Point2D a, b, c;

    //
    // Обратите внимание! В методе Lines(), в отличие от Vertices(), координаты точек замыкаются на начало:
    // a.x, b.x, c.x а затем идёт снова первая вершина a.x
    //
    // Это необходимо для правильного рисования фигур через gnuplot, который формирует линии используя пары точек.
    // В случае с  Triangle будут составлены такие пары точек:
    //      - { a, b }
    //      - { b, c }
    //      - { c, a }
    //
    Point2D Center() { return (a + b + c) / 3.0; }
    std::array<Point2D, 3> Vertices() { return {a, b, c}; }
    Lines2D<4> Lines() const { return {{a.x, b.x, c.x, a.x}, {a.y, b.y, c.y, a.y}}; }

    double Area() { return std::fabs((b - a).Cross(c - a) / 2); }

    double Height() { return std::max({a.y, b.y, c.y}) - std::min({a.y, b.y, c.y}); }

    BoundingBox BoundBox() {
        return BoundingBox{std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}), std::max({a.x, b.x, c.x}),
                           std::max({a.y, b.y, c.y})};
    }
};

struct Rectangle {
    Point2D bottom_left;
    double width, height;

    /* ваш код здесь */
    Point2D Center() { return bottom_left + Point2D{width, height} / 2; }
    std::array<Point2D, 4> Vertices() {
        return {bottom_left, bottom_left + Point2D{width, 0}, bottom_left + Point2D{width, height},
                bottom_left + Point2D{0, height}};
    }
    Lines2D<5> Lines() const {
        return {{bottom_left.x, bottom_left.x + width, bottom_left.x + width, bottom_left.x, bottom_left.x},
                {bottom_left.y, bottom_left.y, bottom_left.x + height, bottom_left.x + height, bottom_left.y}};
    }

    double Area() { return width * height; }

    double Height() { return height; }

    BoundingBox BoundBox() {
        return BoundingBox{bottom_left.x, bottom_left.y, bottom_left.x + width, bottom_left.y + height};
    }
};

struct RegularPolygon {
    Point2D center_p;
    double radius;
    int sides;

    constexpr RegularPolygon(Point2D center, double radius, int sides)
        : center_p(center), radius(radius), sides(sides) {}

    Point2D Center() { return center_p; }
    std::vector<Point2D> Vertices() {
        std::vector<Point2D> points;
        points.reserve(sides);

        for (int i = 0; i < sides; ++i) {
            const double angle = 2 * std::numbers::pi * i / sides;
            points.emplace_back(center_p.x + radius * std::cos(angle), center_p.y + radius * std::sin(angle));
        }
        return points;
    }

    Lines2DDyn Lines() const {
        std::vector<double> x;
        std::vector<double> y;
        x.reserve(sides + 1);
        y.reserve(sides + 1);
        for (int i = 0; i < sides + 1; ++i) {
            const double angle = 2 * std::numbers::pi * i / sides;
            x.emplace_back(center_p.x + radius * std::cos(angle));
            y.emplace_back(center_p.y + radius * std::sin(angle));
        }
        return Lines2DDyn(std::move(x), std::move(y));
    }

    double Height() { return 0; }

    BoundingBox BoundBox() {
        std::vector<Point2D> points = Vertices();
        double min_x = INFINITY;
        double min_y = INFINITY;
        double max_x = -INFINITY;
        double max_y = -INFINITY;
        std::for_each(points.begin(), points.end(), [&](const Point2D &p) {
            if (p.x < min_x) {
                min_x = p.x;
            }
            if (p.y < min_y) {
                min_y = p.y;
            }
            if (p.x > max_x) {
                max_x = p.x;
            }
            if (p.y > max_y) {
                max_y = p.y;
            }
        });
        return BoundingBox{min_x, min_y, max_x, max_y};
    }
};

struct Circle {
    Point2D center_p;
    double radius;

    constexpr Circle(Point2D center, double radius) : center_p(center), radius(radius) {}

    BoundingBox BoundBox() {
        return {center_p.x - radius, center_p.y - radius, center_p.x + radius, center_p.y + radius};
    }
    double Height() { return center_p.y + radius; }
    Point2D Center() { return center_p; }

    //
    // Должны быть сделана по аналогии с RegularPolygon::Vertices
    //
    std::vector<Point2D> Vertices(int N = 30) { return RegularPolygon{center_p, radius, N}.Vertices(); }
    Lines2DDyn Lines(int N = 100) const { return RegularPolygon{center_p, radius, N}.Lines(); }
};

class Polygon {
public:
    /* ваш код здесь */

    //
    // Должны быть сделана по аналогии с RegularPolygon::Vertices
    //
    Point2D Center() {
        return std::accumulate(points_.begin(), points_.end(), Point2D{0.0, 0.0},
                               [](Point2D acc, Point2D current) { return acc + current; }) /
               points_.size();
    }
    std::vector<Point2D> Vertices() const { return points_; }
    Lines2DDyn Lines() const {
        Lines2DDyn lines;
        size_t sz = points_.size();
        lines.Reserve(sz + 1);
        for (int i = 0; i < sz; ++i) {
            lines.x.emplace_back(points_[i].x);
            lines.y.emplace_back(points_[i].y);
        }
        lines.x.emplace_back(points_[0].x);
        lines.y.emplace_back(points_[0].y);
        return lines;
    }

    BoundingBox BoundBox() { return bounding_box_; }

    double Height() { return bounding_box_.Height(); }

private:
    std::vector<Point2D> points_;
    BoundingBox bounding_box_;
};

using Shape = std::variant<Line, Triangle, Rectangle, RegularPolygon, Circle, Polygon>;

enum class GeometryError { Unsupported, NoIntersection, InvalidInput, DegenrateCase, InsufficientPoints };

template <typename T>
using GeometryResult = std::expected<T, GeometryError>;

/*
 * В коде везде используется ReplaceMe. Ваша задача - удалить ReplaceMe и везде вместо него
 использовать наиболее подходящий тип для решения задачи
 */
struct ReplaceMe {
    ReplaceMe(std::vector<Shape>) {}
};

}  // namespace geometry

template <>
struct std::formatter<geometry::Point2D> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Point2D &p, FormatContext &ctx) const {
        return format_to(ctx.out(), "({:.2f}, {:.2f})", p.x, p.y);
    }
};
template <>
struct std::formatter<std::vector<geometry::Point2D>> {
    bool use_new_line = false;

    constexpr auto parse(std::format_parse_context &ctx) {
        auto it = ctx.begin();
        auto end = ctx.end();
        if (it == end || *it == '}') {
            return it;
        }

        const char *format = "new_line";
        int i = 0;
        std::string specifier;
        while (it != end && *it != '}') {
            specifier += *it;
            ++it;
        }
        if (specifier == format) {
            use_new_line = true;
        } else {
            std::format_error("Invalid format specifier");
        }
        return it;
    }

    template <typename FormatContext>
    auto format(const std::vector<geometry::Point2D> &v, FormatContext &ctx) const {
        if (v.empty()) {
            return std::format_to(ctx.out(), "[]");
        }
        auto out = ctx.out();
        *out++ = '[';
        if (use_new_line) {
            *out++ = '\n';
            for (const auto &p : v) {
                *out++ = '\t';
                out = std::format_to(out, "{}", p);
                *out++ = '\n';
            }
        } else {
            for (const auto &p : v) {
                out = std::format_to(out, "{}, ", p);
            }
        }
        *out++ = ']';
        return ctx.out();
    }
};

template <>
struct std::formatter<geometry::Line> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Line &l, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Line({}, {})", l.start, l.end);
    }
};

template <>
struct std::formatter<geometry::Circle> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Circle &c, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Circle(center={}, r={:.2f})", c.center_p, c.radius);
    }
};

template <>
struct std::formatter<geometry::Rectangle> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Rectangle &r, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Rectangle(bottom_left={}, w={:.2f}, h={:.2f})", r.bottom_left, r.width,
                              r.height);
    }
};

template <>
struct std::formatter<geometry::RegularPolygon> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::RegularPolygon &p, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "RegularPolygon(center={}, r={:.2f}, sides={})", p.center_p, p.radius,
                              p.sides);
    }
};
template <>
struct std::formatter<geometry::Triangle> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Triangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Triangle({}, {}, {})", t.a, t.b, t.c);
    }
};
template <>
struct std::formatter<geometry::Polygon> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Polygon &poly, FormatContext &ctx) const {
        auto out = ctx.out();
        out = std::format_to(out, "Polygon[{} points]: [", poly.Vertices().size());

        for (const auto &p : poly.Vertices()) {
            out = std::format_to(out, "{} ", p);
        }

        return std::format_to(out, "]");
    }
};
