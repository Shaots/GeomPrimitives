#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <format>
#include <set>
#include <vector>

namespace geometry::triangulation {

struct DelaunayTriangle {
    Point2D a, b, c;

    DelaunayTriangle(Point2D a, Point2D b, Point2D c) : a(a), b(b), c(c) {}

    bool ContainsPoint(const Point2D &p) const {
        Point2D center = Circumcenter();
        double radius = Circumradius();
        return center.DistanceTo(p) <= radius + geometry::eps;
    }

    Point2D Circumcenter() const {
        double d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
        if (std::abs(d) < geometry::eps) {
            return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3};
        }

        double ux = ((a.x * a.x + a.y * a.y) * (b.y - c.y) + (b.x * b.x + b.y * b.y) * (c.y - a.y) +
                     (c.x * c.x + c.y * c.y) * (a.y - b.y)) /
                    d;

        double uy = ((a.x * a.x + a.y * a.y) * (c.x - b.x) + (b.x * b.x + b.y * b.y) * (a.x - c.x) +
                     (c.x * c.x + c.y * c.y) * (b.x - a.x)) /
                    d;

        return {ux, uy};
    }

    double Circumradius() const {
        Point2D center = Circumcenter();
        return center.DistanceTo(a);
    }

    bool SharesEdge(const DelaunayTriangle &other) const {
        std::vector<Point2D> this_points = {a, b, c};
        std::vector<Point2D> other_points = {other.a, other.b, other.c};

        int shared_count = 0;
        for (const Point2D &p1 : this_points) {
            for (const Point2D &p2 : other_points) {
                if (std::abs(p1.x - p2.x) < geometry::eps && std::abs(p1.y - p2.y) < geometry::eps) {
                    shared_count++;
                    break;
                }
            }
        }

        return shared_count == 2;
    }

    std::vector<Point2D> vertices() const { return {a, b, c}; }
};

struct Edge {
    Point2D p1, p2;

    Edge(Point2D p1, Point2D p2) : p1(p1), p2(p2) {
        if (p1.x > p2.x || (p1.x == p2.x && p1.y > p2.y)) {
            std::swap(this->p1, this->p2);
        }
    }

    bool operator<(const Edge &other) const {
        if (std::abs(p1.x - other.p1.x) > geometry::eps)
            return p1.x < other.p1.x;
        if (std::abs(p1.y - other.p1.y) > geometry::eps)
            return p1.y < other.p1.y;
        if (std::abs(p2.x - other.p2.x) > geometry::eps)
            return p2.x < other.p2.x;
        return p2.y < other.p2.y;
    }

    bool operator==(const Edge &other) const {
        return std::abs(p1.x - other.p1.x) < geometry::eps && std::abs(p1.y - other.p1.y) < geometry::eps &&
               std::abs(p2.x - other.p2.x) < geometry::eps && std::abs(p2.y - other.p2.y) < geometry::eps;
    }
};

inline GeometryResult<std::vector<DelaunayTriangle>> DelaunayTriangulation(std::span<const Point2D> points) {

    if (points.size() < 3) {
        return std::unexpected(GeometryError::InsufficientPoints);
    }
    double min_x = points[0].x;
    double max_x = points[0].x;
    double min_y = points[0].y;
    double max_y = points[0].y;
    for (const auto &p : points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    double dx = max_x - min_x;
    double dy = max_y - min_y;
    double delta = std::max(dx, dy) * 10;

    Point2D super1(min_x - delta, min_y - delta);
    Point2D super2(max_x + delta, min_y - delta);
    Point2D super3(min_x + dx / 2, max_y + delta);

    std::vector<Point2D> super_vertices = {super1, super2, super3};

    std::vector<DelaunayTriangle> triangulation;
    triangulation.emplace_back(super1, super2, super3);

    for (const Point2D &point : points) {
        std::vector<DelaunayTriangle> bad_triangles;
        for (const auto &triangle : triangulation) {
            if (triangle.ContainsPoint(point)) {
                bad_triangles.push_back(triangle);
            }
        }
        std::set<Edge> polygon_edges;

        for (const auto &triangle : bad_triangles) {
            std::vector<Edge> edges = {Edge(triangle.a, triangle.b), Edge(triangle.b, triangle.c),
                                       Edge(triangle.c, triangle.a)};

            for (const auto &edge : edges) {
                auto it = polygon_edges.find(edge);
                if (it != polygon_edges.end()) {
                    polygon_edges.erase(it);
                } else {
                    polygon_edges.insert(edge);
                }
            }
        }

        triangulation.erase(std::remove_if(triangulation.begin(), triangulation.end(),
                                           [&](const DelaunayTriangle &tri) {
                                               return std::find_if(bad_triangles.begin(), bad_triangles.end(),
                                                                   [&](const DelaunayTriangle &bad_tri) {
                                                                       return tri.a == bad_tri.a &&
                                                                              tri.b == bad_tri.b && tri.c == bad_tri.c;
                                                                   }) != bad_triangles.end();
                                           }),
                            triangulation.end());

        for (const auto &edge : polygon_edges) {
            triangulation.emplace_back(edge.p1, edge.p2, point);
        }
    }

    triangulation.erase(std::remove_if(triangulation.begin(), triangulation.end(),
                                       [&](const DelaunayTriangle &tri) {
                                           for (const Point2D &vertex : {tri.a, tri.b, tri.c}) {
                                               for (const Point2D &super_vertex : super_vertices) {
                                                   if (vertex == super_vertex) {
                                                       return true;
                                                   }
                                               }
                                           }
                                           return false;
                                       }),
                        triangulation.end());

    return triangulation;
}
}  // namespace geometry::triangulation

template <>
struct std::formatter<geometry::triangulation::DelaunayTriangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::triangulation::DelaunayTriangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "DelaunayTriangle({}, {}, {})", t.a, t.b, t.c);
    }
};
