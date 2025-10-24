#include <geometry.hpp>
#include <gtest/gtest.h>
#include <intersections.hpp>
#include <queries.hpp>

using namespace geometry;

TEST(BasicCheck, Point2D) {
    Point2D p1(-1, -2);
    Point2D p2(2, 1);
    EXPECT_LT(p1, p2);
}

TEST(BasicCheck, BoundingBox) {
    BoundingBox box(-1, 2, 3, 5);
    EXPECT_EQ(box.Width(), 4);
    EXPECT_EQ(box.Height(), 3);
    EXPECT_EQ(box.Center(), Point2D(1, 3.5));
}

TEST(BasicCheck, Line) {
    Line l(Point2D(-1, -2), Point2D(2, 2));
    EXPECT_EQ(l.length(), 5);
    EXPECT_EQ(l.Direction(), Point2D(3, 4));
    EXPECT_EQ(l.BoundBox(), BoundingBox(-1, -2, 2, 2));
    EXPECT_EQ(l.Center(), Point2D(0.5, 0));
}

TEST(BasicCheck, Triangle) {
    Triangle tr(Point2D(-1, -2), Point2D(1, -3), Point2D(2, 1));
    EXPECT_EQ(tr.Center(), Point2D(2.0 / 3, -4.0 / 3));
    EXPECT_EQ(tr.Area(), 4.5);
    EXPECT_EQ(tr.Height(), 4);
    EXPECT_EQ(tr.BoundBox(), BoundingBox(-1, -3, 2, 1));
}

TEST(BasicCheck, Rectangle) {
    Rectangle rect(Point2D(-1, -2), 3, 4);
    EXPECT_EQ(rect.Center(), Point2D(0.5, 0));
    EXPECT_EQ(rect.Area(), 12);
    EXPECT_EQ(rect.Height(), 4);
    EXPECT_EQ(rect.BoundBox(), BoundingBox(-1, -2, 2, 2));
}

TEST(BasicCheck, RegularPolygon) {
    RegularPolygon regPolygon(Point2D(-1, -2), 3, 10);
    BoundingBox res = BoundingBox(-4, -2 - 3 * std::sin(std::numbers::pi * 2.0 / 5), 2,
                                  -2 + 3 * std::sin(std::numbers::pi * 2.0 / 5));
    EXPECT_DOUBLE_EQ(regPolygon.BoundBox().min_x, res.min_x);
    EXPECT_DOUBLE_EQ(regPolygon.BoundBox().min_y, res.min_y);
    EXPECT_DOUBLE_EQ(regPolygon.BoundBox().max_x, res.max_x);
    EXPECT_DOUBLE_EQ(regPolygon.BoundBox().max_y, res.max_y);
}

TEST(BasicCheck, Circle) {
    Circle circle(Point2D(-1, -2), 5);
    EXPECT_EQ(circle.BoundBox(), BoundingBox(-6, -7, 4, 3));
    EXPECT_EQ(circle.Lines().x.size(), 101);
    EXPECT_EQ(circle.Vertices().size(), 30);
}

TEST(BasicCheck, FormatP) {
    RegularPolygon regPolygon(Point2D(-1, -2), 3, 10);
    std::cout << std::format("{}", regPolygon.Vertices()) << std::endl;
    std::cout << std::format("{:new_line}", regPolygon.Vertices()) << std::endl;
}

class IntersectionLines : public testing::TestWithParam<std::tuple<Shape, Shape, std::vector<Point2D>>> {};

using namespace geometry::intersections;
TEST_P(IntersectionLines, Lines) {
    std::tuple<Shape, Shape, std::vector<Point2D>> input = GetParam();
    EXPECT_EQ(GetIntersectPoint(std::get<0>(input), std::get<1>(input)), std::get<2>(input));
}

INSTANTIATE_TEST_SUITE_P(
    A, IntersectionLines,
    testing::Values(std::make_tuple<Shape, Shape, std::vector<Point2D>>(Line(Point2D(0, 0), Point2D(2, 2)),
                                                                        Line(Point2D(0, -1), Point2D(1, 0)), {}),
                    std::make_tuple<Shape, Shape, std::vector<Point2D>>(Line(Point2D(0, 0), Point2D(2, 2)),
                                                                        Line(Point2D(0, -1), Point2D(1, -2)), {}),
                    std::make_tuple<Shape, Shape, std::vector<Point2D>>(Line(Point2D(0, 0), Point2D(2, 2)),
                                                                        Line(Point2D(2, 0), Point2D(0, 2)),
                                                                        std::vector{Point2D(1, 1)}),
                    std::make_tuple<Shape, Shape, std::vector<Point2D>>(Line(Point2D(0, 0), Point2D(2, 2)),
                                                                        Line(Point2D(2, 2), Point2D(100, 5)),
                                                                        std::vector{Point2D(2, 2)})));

class IntersectionCircles : public testing::TestWithParam<std::tuple<Shape, Shape, std::vector<Point2D>>> {};
TEST_P(IntersectionCircles, Circles) {
    std::tuple<Shape, Shape, std::vector<Point2D>> input = GetParam();
    EXPECT_EQ(GetIntersectPoint(std::get<0>(input), std::get<1>(input)), std::get<2>(input));
}

INSTANTIATE_TEST_SUITE_P(
    A, IntersectionCircles,
    testing::Values(
        std::make_tuple<Shape, Shape, std::vector<Point2D>>(Circle(Point2D(-2, 0), 1), Circle(Point2D(2, 0), 1), {}),
        std::make_tuple<Shape, Shape, std::vector<Point2D>>(Circle(Point2D(-2, 0), 2), Circle(Point2D(2, 0), 2),
                                                            std::vector{Point2D(0, 0)}),
        std::make_tuple<Shape, Shape, std::vector<Point2D>>(Circle(Point2D(0, 0), sqrt(2)),
                                                            Circle(Point2D(2, 2), sqrt(2)), std::vector{Point2D(1, 1)}),
        std::make_tuple<Shape, Shape, std::vector<Point2D>>(Circle(Point2D(0, 0), 1), Circle(Point2D(1, 1), 1),
                                                            std::vector{Point2D(1, 0), Point2D(0, 1)})));

class IntersectionOthers : public testing::TestWithParam<std::pair<Shape, Shape>> {};
TEST_P(IntersectionOthers, Circles) {
    std::pair<Shape, Shape> input = GetParam();
    EXPECT_THROW(GetIntersectPoint(input.first, input.second), std::logic_error);
}

INSTANTIATE_TEST_SUITE_P(
    A, IntersectionOthers,
    testing::Values(std::make_pair<Shape, Shape>(Circle(Point2D(-2, 0), 1), Line(Point2D(0, 0), Point2D(2, 2))),
                    std::make_pair<Shape, Shape>(Line(Point2D(0, 0), Point2D(2, 2)), Circle(Point2D(-2, 0), 1)),
                    std::make_pair<Shape, Shape>(RegularPolygon(Point2D(-1, -2), 5, 6),
                                                 Triangle(Point2D(-1, -2), Point2D(1, -3), Point2D(2, 1))),
                    std::make_pair<Shape, Shape>(RegularPolygon(Point2D(-1, -2), 5, 6),
                                                 Line(Point2D(0, 0), Point2D(2, 2)))));

class Distance2Point : public testing::TestWithParam<std::tuple<Shape, Point2D, double>> {};
TEST_P(Distance2Point, A) {
    using namespace geometry::queries;
    std::tuple<Shape, Point2D, double> input = GetParam();
    EXPECT_DOUBLE_EQ(DistanceToPoint(std::get<0>(input), std::get<1>(input)), std::get<2>(input));
}

INSTANTIATE_TEST_SUITE_P(
    A, Distance2Point,
    testing::Values(
        std::make_tuple<Shape, Point2D, double>(Line(Point2D(0, 0), Point2D(2, 2)), Point2D(0, 0), 0),
        std::make_tuple<Shape, Point2D, double>(Line(Point2D(0, 0), Point2D(2, 2)), Point2D(1, 1), 0),
        std::make_tuple<Shape, Point2D, double>(Line(Point2D(0, 0), Point2D(2, 2)), Point2D(1, 0), sqrt(2) / 2),
        std::make_tuple<Shape, Point2D, double>(Line(Point2D(0, 0), Point2D(2, 2)), Point2D(0, 1), sqrt(2) / 2),
        std::make_tuple<Shape, Point2D, double>(Line(Point2D(0, 0), Point2D(2, 2)), Point2D(-1, -1), sqrt(2)),
        std::make_tuple<Shape, Point2D, double>(Line(Point2D(0, 0), Point2D(2, 2)), Point2D(-2, 1), sqrt(5)),
        std::make_tuple<Shape, Point2D, double>(Triangle(Point2D(0, 0), Point2D(2, 2), Point2D(0, 4)), Point2D(-2, 1),
                                                2),
        std::make_tuple<Shape, Point2D, double>(Triangle(Point2D(0, 0), Point2D(2, 2), Point2D(0, 4)), Point2D(1, 2),
                                                sqrt(2) / 2),
        std::make_tuple<Shape, Point2D, double>(Rectangle(Point2D(0, 0), 5, 6), Point2D(3, 4), 0),
        std::make_tuple<Shape, Point2D, double>(Rectangle(Point2D(0, 0), 5, 6), Point2D(-1, -1), sqrt(2)),
        std::make_tuple<Shape, Point2D, double>(Rectangle(Point2D(0, 0), 5, 6), Point2D(-1, 2), 1),
        std::make_tuple<Shape, Point2D, double>(RegularPolygon(Point2D(0, 0), 6, 6), Point2D(3, 0), 3 * sqrt(3) / 2),
        std::make_tuple<Shape, Point2D, double>(RegularPolygon(Point2D(0, 0), 6, 6), Point2D(0, 8), 8 - 3 * sqrt(3)),
        std::make_tuple<Shape, Point2D, double>(Circle(Point2D(0, 0), 6), Point2D(3, 0), 0),
        std::make_tuple<Shape, Point2D, double>(Circle(Point2D(0, 0), 6), Point2D(0, 8), 2)));

class DistanceShape2Shape : public testing::TestWithParam<std::tuple<Shape, Shape, std::optional<double>>> {};
TEST_P(DistanceShape2Shape, A) {
    using namespace geometry::queries;
    std::tuple<Shape, Shape, std::optional<double>> input = GetParam();
    EXPECT_DOUBLE_EQ(DistanceBetweenShapes(std::get<0>(input), std::get<1>(input)).value(), std::get<2>(input).value());
}

INSTANTIATE_TEST_SUITE_P(
    A, DistanceShape2Shape,
    testing::Values(
        std::make_tuple<Shape, Shape, std::optional<double>>(Line(Point2D(0, 0), Point2D(2, 2)),
                                                             Line(Point2D(2, 0), Point2D(0, 2)), 0.0),
        std::make_tuple<Shape, Shape, std::optional<double>>(Line(Point2D(0, 0), Point2D(2, 2)),
                                                             Line(Point2D(2, 0), Point2D(3, -1)), sqrt(2)),
        std::make_tuple<Shape, Shape, std::optional<double>>(Circle(Point2D(0, 0), 6), Circle(Point2D(10, 0), 5), 0),
        std::make_tuple<Shape, Shape, std::optional<double>>(Circle(Point2D(0, 0), 4), Circle(Point2D(10, 0), 3), 3),
        std::make_tuple<Shape, Shape, std::optional<double>>(Circle(Point2D(0, 0), 20), Circle(Point2D(5, 0), 10), 0)));