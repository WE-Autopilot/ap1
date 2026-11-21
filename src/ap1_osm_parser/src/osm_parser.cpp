#include "ap1_osm_parser/osm_parser.hpp"

#include <tinyxml2.h>
#include <stdexcept>

namespace ap1_osm_parser
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

void OsmParser::loadOsm(const std::string & filepath)
{
  nodes_.clear();
  ways_.clear();
  way_list_.clear();
  relations_.clear();

  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(filepath.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to load OSM file: " + filepath);
  }

  const auto * root = doc.FirstChildElement("osm");
  if (!root) {
    throw std::runtime_error("Invalid OSM file (missing <osm> root): " + filepath);
  }

  // Parse <node> elements
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
    nodes_[node.id] = std::move(node);
  }

  // Parse <way> elements
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
    ways_[way.id] = way;
    way_list_.push_back(std::move(way));
  }

  // Parse <relation> elements
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
    relations_.push_back(std::move(rel));
  }
}

std::vector<Coordinate> OsmParser::getWayCoordinates(const std::string & way_id) const
{
  const auto way_it = ways_.find(way_id);
  if (way_it == ways_.end()) {
    throw std::out_of_range("Way ID not found: " + way_id);
  }

  std::vector<Coordinate> coords;
  coords.reserve(way_it->second.node_refs.size());

  for (const auto & node_ref : way_it->second.node_refs) {
    const auto node_it = nodes_.find(node_ref);
    if (node_it == nodes_.end()) {
      throw std::out_of_range("Node ID not found: " + node_ref);
    }
    coords.push_back({node_it->second.lat, node_it->second.lon});
  }
  return coords;
}

std::vector<Relation> OsmParser::getLanelets() const
{
  std::vector<Relation> lanelets;

  for (const auto & rel : relations_) {
    const auto type_it = rel.tags.find("type");
    if (type_it == rel.tags.end() || type_it->second != "lanelet") {
      continue;
    }

    Relation resolved = rel;
    for (auto & member : resolved.members) {
      if (member.type == "way") {
        const auto way_it = ways_.find(member.id);
        if (way_it != ways_.end()) {
          member.tags = way_it->second.tags;
          member.coordinates = getWayCoordinates(member.id);
        }
      }
    }
    lanelets.push_back(std::move(resolved));
  }
  return lanelets;
}

std::vector<FeatureWay> OsmParser::getWaysByType(const std::string & feature_type) const
{
  std::vector<FeatureWay> results;

  for (const auto & way : way_list_) {
    const auto type_it = way.tags.find("type");
    if (type_it != way.tags.end() && type_it->second == feature_type) {
      FeatureWay feature;
      feature.id = way.id;
      feature.type = feature_type;
      feature.tags = way.tags;
      feature.coordinates = getWayCoordinates(way.id);
      results.push_back(std::move(feature));
    }
  }
  return results;
}

}  // namespace ap1_osm_parser