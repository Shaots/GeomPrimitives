#include <geometry.hpp>
#include <gtest/gtest.h>

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