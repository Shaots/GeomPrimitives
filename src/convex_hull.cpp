#include "convex_hull.hpp"
#include <algorithm>

namespace geometry::convex_hull {

double CrossProduct(Point2D p1, Point2D middle, Point2D p2) {
    auto new_p1 = p1 - middle;
    auto new_p2 = p2 - middle;
    return new_p1.Cross(new_p2);
}

GeometryResult<std::vector<Point2D>> GrahamScan(std::vector<Point2D> points) {
    if (points.size() < 3) {
        return std::unexpected{GeometryError::InsufficientPoints};
    }
    auto p0_it = std::ranges::min_element(points, [](const Point2D &a, const Point2D &b) {
        if (a.y != b.y) {
            return a.y < b.y;
        }
        return a.x < b.x;
    });
    Point2D p0 = *p0_it;
    std::iter_swap(points.begin(), p0_it);
    std::ranges::sort(points.begin() + 1, points.end(), [p0](const Point2D &a, const Point2D &b) {
        double cross = CrossProduct(a, p0, b);
        if (std::fabs(cross) < geometry::eps) {
            return p0.DistanceTo(a) > p0.DistanceTo(b);
        }
        return cross > 0;
    });
    auto unique_pred = [p0](const Point2D &a, const Point2D &b) {
        double cross = CrossProduct(a, p0, b);
        return std::fabs(cross) < geometry::eps;
    };
    auto last = std::unique(points.begin() + 1, points.end(), unique_pred);
    points.erase(last, points.end());
    if (points.size() < 3) {
        return std::unexpected{GeometryError::InsufficientPoints};
    }
    StackForGrahamScan stack;
    stack.Push(points[0]);
    stack.Push(points[1]);
    stack.Push(points[2]);
    for (size_t i = 3; i < points.size(); ++i) {
        while (stack.Size() > 1) {
            Point2D top = stack.Top();
            Point2D next_to_top = stack.NextToTop();
            if (CrossProduct(next_to_top, top, points[i]) <= 0) {
                stack.Pop();
            } else {
                break;
            }
        }
        stack.Push(points[i]);
    }
    auto hull = std::move(stack).Extract();
    if (hull.size() < 3) {
        return std::unexpected{GeometryError::DegenrateCase};
    }

    return hull;
}

}  // namespace geometry::convex_hull