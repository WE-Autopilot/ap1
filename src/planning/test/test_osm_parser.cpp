#include <gtest/gtest.h>
#include <string>
#include "osm_parser.hpp"

using planning::util::getWayCoordinates;
using planning::util::getWaysByType;
using planning::util::parseOSM;

class OsmParserTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    sample_path_ = std::string(TEST_FIXTURE_DIR) + "/sample_map.osm";
  }

  std::string sample_path_;
};

TEST_F(OsmParserTest, ThrowsOnMissingFile)
{
  EXPECT_THROW(parseOSM("/nonexistent/path/map.osm"), std::runtime_error);
}

TEST_F(OsmParserTest, LoadsCountsCorrectly)
{
  auto map = parseOSM(sample_path_);
  EXPECT_EQ(map.nodes.size(), 6u);
  EXPECT_EQ(map.ways.size(), 4u);
  EXPECT_EQ(map.relations.size(), 2u);
}

TEST_F(OsmParserTest, ResolvesWayCoordinatesInOrder)
{
  auto map = parseOSM(sample_path_);
  auto coords = getWayCoordinates(map, "100");

  ASSERT_EQ(coords.size(), 2u);
  EXPECT_DOUBLE_EQ(coords[0].lat, 43.0096);
  EXPECT_DOUBLE_EQ(coords[0].lon, -81.2737);
  EXPECT_DOUBLE_EQ(coords[1].lat, 43.0097);
  EXPECT_DOUBLE_EQ(coords[1].lon, -81.2737);
}

TEST_F(OsmParserTest, ExtractsLaneletsAndResolvesMembers)
{
  auto map = parseOSM(sample_path_);

  ASSERT_EQ(map.lanelets.size(), 1u);
  EXPECT_EQ(map.lanelets[0].id, "200");
  EXPECT_EQ(map.lanelets[0].tags.at("subtype"), "road");
  ASSERT_EQ(map.lanelets[0].members.size(), 3u);

  const auto & centerline = map.lanelets[0].members[2];
  EXPECT_EQ(centerline.role, "centerline");
  ASSERT_EQ(centerline.coordinates.size(), 2u);
  EXPECT_DOUBLE_EQ(centerline.coordinates[0].lon, -81.27365);
}

TEST_F(OsmParserTest, FiltersWaysByFeatureType)
{
  auto map = parseOSM(sample_path_);

  auto curbstones = getWaysByType(map, "curbstone");
  ASSERT_EQ(curbstones.size(), 1u);
  EXPECT_EQ(curbstones[0].id, "101");

  auto stop_lines = getWaysByType(map, "stop_line");
  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines[0].id, "103");

  auto road_borders = getWaysByType(map, "road_border");
  EXPECT_TRUE(road_borders.empty());
}
