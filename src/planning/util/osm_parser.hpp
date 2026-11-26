#ifndef PLANNING__UTIL__OSM_PARSER_HPP_
#define PLANNING__UTIL__OSM_PARSER_HPP_

#include <string>
#include <unordered_map>
#include <vector>

namespace planning::util
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

struct Map
{
  std::unordered_map<std::string, Node> nodes;
  std::unordered_map<std::string, Way> ways;
  std::vector<Way> way_list;
  std::vector<Relation> relations;
  std::vector<Relation> lanelets;
};

Map parseOSM(const std::string & filepath);
std::vector<Coordinate> getWayCoordinates(const Map & map, const std::string & way_id);
std::vector<FeatureWay> getWaysByType(const Map & map, const std::string & feature_type);

}  // namespace planning::util

#endif  // PLANNING__UTIL__OSM_PARSER_HPP_
