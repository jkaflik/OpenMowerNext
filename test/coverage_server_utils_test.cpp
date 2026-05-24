#include "coverage_server/utils.h"

#include <gtest/gtest.h>

namespace
{
geometry_msgs::msg::PolygonStamped makePolygon(
  std::initializer_list<std::pair<double, double>> points)
{
  geometry_msgs::msg::PolygonStamped polygon;
  polygon.header.frame_id = "map";

  for (const auto & [x, y] : points) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    polygon.polygon.points.push_back(point);
  }

  return polygon;
}
}  // namespace

TEST(CoverageServerUtils, EmptyPolygonDoesNotThrowWhenConvertedToLinearRing)
{
  geometry_msgs::msg::Polygon polygon;

  EXPECT_NO_THROW({
    const auto ring = open_mower_next::coverage_server::utils::toLinearRing(polygon);
    EXPECT_TRUE(ring.isEmpty());
  });
}

TEST(CoverageServerUtils, OutsideExclusionIsIgnored)
{
  const auto field = makePolygon({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
  const auto outside_exclusion =
    makePolygon({{20.0, 20.0}, {22.0, 20.0}, {22.0, 22.0}, {20.0, 22.0}});

  const auto cells = open_mower_next::coverage_server::utils::toCells(field, {outside_exclusion});

  EXPECT_EQ(cells.size(), 1u);
  EXPECT_NEAR(cells.area(), 100.0, 1e-6);
}

TEST(CoverageServerUtils, InsideExclusionCreatesHole)
{
  const auto field = makePolygon({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
  const auto inside_exclusion = makePolygon({{2.0, 2.0}, {4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}});

  const auto cells = open_mower_next::coverage_server::utils::toCells(field, {inside_exclusion});

  EXPECT_EQ(cells.size(), 1u);
  EXPECT_NEAR(cells.area(), 96.0, 1e-6);
}

TEST(CoverageServerUtils, PartialOverlapExclusionClipsField)
{
  const auto field = makePolygon({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
  const auto overlapping_exclusion =
    makePolygon({{8.0, 2.0}, {12.0, 2.0}, {12.0, 4.0}, {8.0, 4.0}});

  const auto cells =
    open_mower_next::coverage_server::utils::toCells(field, {overlapping_exclusion});

  EXPECT_EQ(cells.size(), 1u);
  EXPECT_NEAR(cells.area(), 96.0, 1e-6);
}

TEST(CoverageServerUtils, MultipleExclusionsAreApplied)
{
  const auto field = makePolygon({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
  const auto inside_exclusion = makePolygon({{2.0, 2.0}, {4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}});
  const auto overlapping_exclusion =
    makePolygon({{8.0, 2.0}, {12.0, 2.0}, {12.0, 4.0}, {8.0, 4.0}});
  const auto outside_exclusion =
    makePolygon({{20.0, 20.0}, {22.0, 20.0}, {22.0, 22.0}, {20.0, 22.0}});

  const auto cells = open_mower_next::coverage_server::utils::toCells(
    field, {inside_exclusion, overlapping_exclusion, outside_exclusion});

  EXPECT_EQ(cells.size(), 1u);
  EXPECT_NEAR(cells.area(), 92.0, 1e-6);
}

TEST(CoverageServerUtils, InsideExclusionConvertsToCoverageGeometryHole)
{
  const auto field = makePolygon({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
  const auto inside_exclusion = makePolygon({{2.0, 2.0}, {4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}});
  const auto cells = open_mower_next::coverage_server::utils::toCells(field, {inside_exclusion});

  const auto msg = open_mower_next::coverage_server::utils::toMsg(cells, "map");

  ASSERT_EQ(msg.cells.size(), 1u);
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.cells[0].header.frame_id, "map");
  EXPECT_EQ(msg.cells[0].exterior.points.size(), 4u);
  ASSERT_EQ(msg.cells[0].holes.size(), 1u);
  EXPECT_EQ(msg.cells[0].holes[0].points.size(), 4u);
}

TEST(CoverageServerUtils, PartialOverlapConvertsToClippedCoverageGeometry)
{
  const auto field = makePolygon({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
  const auto overlapping_exclusion =
    makePolygon({{8.0, 2.0}, {12.0, 2.0}, {12.0, 4.0}, {8.0, 4.0}});
  const auto cells =
    open_mower_next::coverage_server::utils::toCells(field, {overlapping_exclusion});

  const auto msg = open_mower_next::coverage_server::utils::toMsg(cells, "map");

  ASSERT_EQ(msg.cells.size(), 1u);
  EXPECT_TRUE(msg.cells[0].holes.empty());
  EXPECT_GE(msg.cells[0].exterior.points.size(), 4u);
}

TEST(CoverageServerUtils, SplittingExclusionConvertsToMultipleCoverageCells)
{
  const auto field = makePolygon({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
  const auto splitting_exclusion =
    makePolygon({{4.0, -1.0}, {6.0, -1.0}, {6.0, 11.0}, {4.0, 11.0}});
  const auto cells = open_mower_next::coverage_server::utils::toCells(field, {splitting_exclusion});

  const auto msg = open_mower_next::coverage_server::utils::toMsg(cells, "map");

  EXPECT_EQ(cells.size(), 2u);
  ASSERT_EQ(msg.cells.size(), 2u);
  EXPECT_TRUE(msg.cells[0].holes.empty());
  EXPECT_TRUE(msg.cells[1].holes.empty());
  EXPECT_NEAR(cells.area(), 80.0, 1e-6);
}
