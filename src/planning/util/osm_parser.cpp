#include "osm_parser.hpp"

#include <tinyxml2.h>
#include <stdexcept>

namespace planning::util
{

namespace
{

std::unordered_map<std::string, std::string> parseTags(const tinyxml2::XMLElement * element)
{
  std::unordered_map<std::string, std::string> tags;
  for (const auto * tag = element->FirstChildElement("tag");
       tag != nullptr;
       tag = tag->NextSiblingElement("tag"))
  {
    const char * k = tag->Attribute("k");
    const char * v = tag->Attribute("v");
    if (k && v) {
      tags[k] = v;
    }
  }
  return tags;
}

}  // namespace

std::vector<Coordinate> getWayCoordinates(const Map & map, const std::string & way_id)
{
  const auto way_it = map.ways.find(way_id);
  if (way_it == map.ways.end()) {
    throw std::out_of_range("Way ID not found: " + way_id);
  }

  std::vector<Coordinate> coords;
  coords.reserve(way_it->second.node_refs.size());

  for (const auto & node_ref : way_it->second.node_refs) {
    const auto node_it = map.nodes.find(node_ref);
    if (node_it == map.nodes.end()) {
      throw std::out_of_range("Node ID not found: " + node_ref);
    }
    coords.push_back({node_it->second.lat, node_it->second.lon});
  }
  return coords;
}

Map parseOSM(const std::string & filepath)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(filepath.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to load OSM file: " + filepath);
  }

  const auto * root = doc.FirstChildElement("osm");
  if (!root) {
    throw std::runtime_error("Invalid OSM file (missing <osm> root): " + filepath);
  }

  Map map;

  for (const auto * elem = root->FirstChildElement("node");
       elem != nullptr;
       elem = elem->NextSiblingElement("node"))
  {
    const char * id = elem->Attribute("id");
    if (!id) {
      continue;
    }
    Node node;
    node.id = id;
    elem->QueryDoubleAttribute("lat", &node.lat);
    elem->QueryDoubleAttribute("lon", &node.lon);
    node.tags = parseTags(elem);
    map.nodes[node.id] = std::move(node);
  }

  for (const auto * elem = root->FirstChildElement("way");
       elem != nullptr;
       elem = elem->NextSiblingElement("way"))
  {
    const char * id = elem->Attribute("id");
    if (!id) {
      continue;
    }
    Way way;
    way.id = id;
    for (const auto * nd = elem->FirstChildElement("nd");
         nd != nullptr;
         nd = nd->NextSiblingElement("nd"))
    {
      if (const char * ref = nd->Attribute("ref")) {
        way.node_refs.emplace_back(ref);
      }
    }
    way.tags = parseTags(elem);
    map.ways[way.id] = way;
    map.way_list.push_back(std::move(way));
  }

  for (const auto * elem = root->FirstChildElement("relation");
       elem != nullptr;
       elem = elem->NextSiblingElement("relation"))
  {
    const char * id = elem->Attribute("id");
    if (!id) {
      continue;
    }
    Relation rel;
    rel.id = id;
    rel.tags = parseTags(elem);

    for (const auto * mem = elem->FirstChildElement("member");
         mem != nullptr;
         mem = mem->NextSiblingElement("member"))
    {
      RelationMember member;
      if (const char * t = mem->Attribute("type")) { member.type = t; }
      if (const char * r = mem->Attribute("ref"))  { member.id = r; }
      if (const char * ro = mem->Attribute("role")) { member.role = ro; }
      rel.members.push_back(std::move(member));
    }

    const auto type_it = rel.tags.find("type");
    if (type_it != rel.tags.end() && type_it->second == "lanelet") {
      Relation lanelet = rel;
      for (auto & member : lanelet.members) {
        if (member.type == "way") {
          const auto way_it = map.ways.find(member.id);
          if (way_it != map.ways.end()) {
            member.tags = way_it->second.tags;
            member.coordinates = getWayCoordinates(map, member.id);
          }
        }
      }
      map.lanelets.push_back(std::move(lanelet));
    }

    map.relations.push_back(std::move(rel));
  }

  return map;
}

std::vector<FeatureWay> getWaysByType(const Map & map, const std::string & feature_type)
{
  std::vector<FeatureWay> results;

  for (const auto & way : map.way_list) {
    const auto type_it = way.tags.find("type");
    if (type_it != way.tags.end() && type_it->second == feature_type) {
      FeatureWay feature;
      feature.id = way.id;
      feature.type = feature_type;
      feature.tags = way.tags;
      feature.coordinates = getWayCoordinates(map, way.id);
      results.push_back(std::move(feature));
    }
  }
  return results;
}

}  // namespace planning::util
