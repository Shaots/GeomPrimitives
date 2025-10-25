#include "convex_hull.hpp"
#include "geometry.hpp"
#include "intersections.hpp"
#include "queries.hpp"
#include "shape_utils.hpp"
#include "triangulation.hpp"
#include "visualization.hpp"

#include <algorithm>
#include <print>
#include <ranges>

using namespace geometry;

namespace rng = std::ranges;
namespace views = std::ranges::views;

void PrintAllIntersections(const Shape &shape, const std::vector<Shape> &others) {
    std::println("\n=== Intersections ===");
    auto supported_shapes = others | views::filter([&](const Shape &other) {
                                try {
                                    auto result = intersections::GetIntersectPoint(shape, other);
                                    return true;
                                } catch (const std::logic_error &) {
                                    return false;
                                }
                            });
    for (const auto &other : supported_shapes) {
        auto result = intersections::GetIntersectPoint(shape, other);
        if (!result.empty()) {
            std::println("The intersection is found at point {} between figures {} and {}", result.front(), shape,
                         other);
        } else {
            std::println("The figures {} and {} do not intersect", shape, other);
        }
    }
}

void PrintDistancesFromPointToShapes(Point2D p, const std::vector<Shape> &shapes) {
    std::println("\n=== Distance from Point Test ===");
    std::println("Testing point: {} ", p);

    /*
     * Используйте ranges чтобы выбрать любые 5 фигур из списка.
     * Затем найдите расстояния от заданной точки до всех выбранных фигур.
     * Выведите результат в формате "Расстояние от точки P до фигуры S равно D"
     */
    auto selected_shapes = shapes | views::take(5) | views::transform([&](const Shape &shape) {
                               double distance = queries::DistanceToPoint(shape, p);
                               return std::make_pair(shape, distance);
                           });

    for (const auto &[shape, distance] : selected_shapes) {
        std::println("The distance from point {} to figure {} is equal to {:.4f}", p, shape, distance);
    }
}

void PerformShapeAnalysis(const std::vector<Shape> &shapes) {
    std::println("\n=== Shape Analysis ===");

    /*
     * Используйте ranges и созданные классы чтобы:
     *     - Найти все пересечения между фигурами используя метод Bounding Box
     *     - Найти самую высокую фигуру (чья высота наибольшая)
     *     - Вывести расстояние между любыми двумя фигурами, которые поддерживают данную функциональность
     */
    auto collisions = utils::FindAllCollisions(shapes);
    std::println("{} Bounding Box intersections found:", collisions.size());
    for (const auto &[shape1, shape2] : collisions) {
        std::println("  - {} and {}", shape1, shape2);
    }

    auto highest_index = utils::FindHighestShape(shapes);
    if (highest_index) {
        std::println("Tallest figure (index {}): {} with height {:.2f}", *highest_index, shapes[*highest_index],
                     queries::GetHeight(shapes[*highest_index]));
    }

    if (shapes.size() >= 2) {
        auto distance_result = queries::DistanceBetweenShapes(shapes[0], shapes[1]);
        if (distance_result) {
            std::println("The distance between the figures {} and {} is {:.2f}", shapes[0], shapes[1],
                         *distance_result);
        } else {
            std::println("Spacing between shapes {} and {} is not supported", shapes[0], shapes[1]);
        }
    }
}

void PerformExtraShapeAnalysis(std::span<const Shape> shapes) {
    std::println("\n=== Shape Extra Analysis ===");

    /*
     * Используйте ranges и созданные классы чтобы:
     *     - Вывести 3 любые фигуры, которые находятся выше 50.0
     *     - Вывести фигуры с наименьшей и с наибольшей высотами
     */
    auto high_shapes =
        shapes | views::filter([](const Shape &shape) { return queries::GetHeight(shape) > 50.0; }) | views::take(3);

    std::println("3 figures above 50.0:");
    for (const auto &shape : high_shapes) {
        std::println("  - {} (height: {:.2f})", shape, queries::GetHeight(shape));
    }

    auto min_height_it = rng::min_element(
        shapes, [](const Shape &a, const Shape &b) { return queries::GetHeight(a) < queries::GetHeight(b); });

    if (min_height_it != shapes.end()) {
        std::println("The figure with the smallest height is: {} (height: {:.2f})", *min_height_it,
                     queries::GetHeight(*min_height_it));
    }

    auto max_height_it = rng::max_element(
        shapes, [](const Shape &a, const Shape &b) { return queries::GetHeight(a) < queries::GetHeight(b); });

    if (max_height_it != shapes.end()) {
        std::println("The figure with the greatest height is: {} (height: {:.2f})", *max_height_it,
                     queries::GetHeight(*max_height_it));
    }
}

int main() {
    utils::ShapeGenerator generator(-50.0, 50.0, 5.0, 25.0);

    //
    // После реализации всех фигур, замените GenerateTriangles на GenerateShapes
    //
    std::vector<Shape> shapes = generator.GenerateTriangles(15);

    std::println("Generated {} random shapes", shapes.size());

    // Выведите индекс каждой фигуры и её высоту

    //
    // Вызываем разработанные функции
    //
    PrintAllIntersections(shapes[0], shapes);

    PrintDistancesFromPointToShapes(Point2D{10.0, 10.0}, shapes);

    PerformShapeAnalysis(shapes);

    PerformExtraShapeAnalysis(shapes);

    //
    // Рисуем все фигуры
    //
    // Важно: после изучения графика - нажмите Enter чтобы продолжить выполнение и построить 2ой график
    //
    // geometry::visualization::Draw(shapes);

    //
    // Формируем список из вершин всех фигур
    //
    std::vector<Point2D> points;

    auto convex_hull_result = convex_hull::GrahamScan(points);
    if (convex_hull_result.has_value()) {
        auto convex_hull_points = convex_hull_result.value();
        Polygon convex_hull_polygon;
        shapes.push_back(convex_hull_polygon);
        std::println("The convex hull is constructed successfully and contains {} points.", convex_hull_points.size());
    } else {
        std::println("Convex hull construction error");
    }

    // geometry::visualization::Draw(shapes);

    //
    // после изучения графика - нажмите Enter чтобы продолжить выполнение и построить 3ий график
    //

    {
        std::vector<Point2D> points = {{0, 0}, {10, 0}, {5, 8}, {15, 5}, {2, 12}};

        //
        // Используйте список точек points или свой, чтобы
        // выполнить алгоритм триангуляции Делоне алгоритмом Боуэра-Ватсона
        //
        // После успешного завершения алгоритма - выведите результат для проверки
        // используя geometry::visualization::Draw
        //
        auto triangulation_result = triangulation::DelaunayTriangulation(points);
        if (triangulation_result.has_value()) {
            auto triangles = triangulation_result.value();
            std::println("The Delaunay triangulation is successfully constructed and contains {} triangles.",
                         triangles.size());
            std::vector<Shape> triangle_shapes;
            for (const auto &triangle : triangles) {
                triangle_shapes.push_back(Triangle{triangle.a, triangle.b, triangle.c});
            }
            // geometry::visualization::Draw(triangle_shapes);
        } else {
            std::println("Delaunay triangulation error");
        }
    }
    return 0;
}