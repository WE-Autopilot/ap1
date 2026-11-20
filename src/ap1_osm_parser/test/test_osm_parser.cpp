#include <gtest/gtest.h>
#include <string>
#include "ap1_osm_parser/osm_parser.hpp"

using ap1_osm_parser::OsmParser;

class OsmParserTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    sample_path_ = std::string(TEST_FIXTURE_DIR) + "/sample_map.osm";
  }

  OsmParser parser_;
  std::string sample_path_;
};

TEST_F(OsmParserTest, ThrowsOnMissingFile)
{
  EXPECT_THROW(parser_.loadOsm("/nonexistent/path/map.osm"), std::runtime_error);
}

TEST_F(OsmParserTest, LoadsCountsCorrectly)
{
  ASSERT_NO_THROW(parser_.loadOsm(sample_path_));
  EXPECT_EQ(parser_.nodeCount(), 6u);
  EXPECT_EQ(parser_.wayCount(), 4u);
  EXPECT_EQ(parser_.relationCount(), 2u);
}

TEST_F(OsmParserTest, ResolvesWayCoordinatesInOrder)
{
  parser_.loadOsm(sample_path_);
  auto coords = parser_.getWayCoordinates("100");

  ASSERT_EQ(coords.size(), 2u);
  EXPECT_DOUBLE_EQ(coords[0].lat, 43.0096);
  EXPECT_DOUBLE_EQ(coords[0].lon, -81.2737);
  EXPECT_DOUBLE_EQ(coords[1].lat, 43.0097);
  EXPECT_DOUBLE_EQ(coords[1].lon, -81.2737);
}

TEST_F(OsmParserTest, ExtractsLaneletsAndResolvesMembers)
{
  parser_.loadOsm(sample_path_);
  auto lanelets = parser_.getLanelets();

  ASSERT_EQ(lanelets.size(), 1u);
  EXPECT_EQ(lanelets[0].id, "200");
  EXPECT_EQ(lanelets[0].tags.at("subtype"), "road");
  ASSERT_EQ(lanelets[0].members.size(), 3u);

  // Verify centerline coordinates resolved
  const auto & centerline = lanelets[0].members[2];
  EXPECT_EQ(centerline.role, "centerline");
  ASSERT_EQ(centerline.coordinates.size(), 2u);
  EXPECT_DOUBLE_EQ(centerline.coordinates[0].lon, -81.27365);
}

TEST_F(OsmParserTest, FiltersWaysByFeatureType)
{
  parser_.loadOsm(sample_path_);

  auto curbstones = parser_.getWaysByType("curbstone");
  ASSERT_EQ(curbstones.size(), 1u);
  EXPECT_EQ(curbstones[0].id, "101");

  auto stop_lines = parser_.getWaysByType("stop_line");
  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines[0].id, "103");

  auto road_borders = parser_.getWaysByType("road_border");
  EXPECT_TRUE(road_borders.empty());
}