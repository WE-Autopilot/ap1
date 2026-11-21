#ifndef AP1_OSM_PARSER__OSM_PARSER_HPP_
#define AP1_OSM_PARSER__OSM_PARSER_HPP_

#include <string>
#include <unordered_map>
#include <vector>

namespace ap1_osm_parser
{

struct Coordinate
{
  double lat{0.0};
  double lon{0.0};
};

struct Node
{
  std::string id;
  double lat{0.0};
  double lon{0.0};
  std::unordered_map<std::string, std::string> tags;
};

struct Way
{
  std::string id;
  std::vector<std::string> node_refs;
  std::unordered_map<std::string, std::string> tags;
};

struct RelationMember
{
  std::string type;
  std::string id;
  std::string role;
  std::unordered_map<std::string, std::string> tags;
  std::vector<Coordinate> coordinates;
};

struct Relation
{
  std::string id;
  std::unordered_map<std::string, std::string> tags;
  std::vector<RelationMember> members;
};

struct FeatureWay
{
  std::string id;
  std::string type;
  std::unordered_map<std::string, std::string> tags;
  std::vector<Coordinate> coordinates;
};

class OsmParser
{
public:
  // Loads and parses an OSM XML file. Throws std::runtime_error on failure.
  void loadOsm(const std::string & filepath);

  // Resolves a way's node references into ordered (lat, lon) coordinates.
  std::vector<Coordinate> getWayCoordinates(const std::string & way_id) const;

  // Extracts all relations with tag type="lanelet" and resolves way members.
  std::vector<Relation> getLanelets() const;

  // Extracts all ways matching tag type=<feature_type>.
  std::vector<FeatureWay> getWaysByType(const std::string & feature_type) const;

  std::size_t nodeCount() const { return nodes_.size(); }
  std::size_t wayCount() const { return ways_.size(); }
  std::size_t relationCount() const { return relations_.size(); }

private:
  std::unordered_map<std::string, Node> nodes_;
  std::unordered_map<std::string, Way> ways_;
  std::vector<Way> way_list_;
  std::vector<Relation> relations_;
};

}  // namespace ap1_osm_parser

#endif  // AP1_OSM_PARSER__OSM_PARSER_HPP_