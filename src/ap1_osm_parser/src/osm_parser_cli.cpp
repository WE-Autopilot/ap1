#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "ap1_osm_parser/osm_parser.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: osm_parser_cli <path_to_osm_file>\n";
    return 1;
  }

  ap1_osm_parser::OsmParser parser;
  try {
    parser.loadOsm(argv[1]);
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  std::cout << "OSM file loaded successfully!\n";
  std::cout << "Nodes: " << parser.nodeCount() << "\n";
  std::cout << "Ways: " << parser.wayCount() << "\n";
  std::cout << "Relations: " << parser.relationCount() << "\n";

  auto lanelets = parser.getLanelets();
  std::cout << "\nTotal lanelets: " << lanelets.size() << "\n";

  for (const std::string & feature : {"line_thin", "road_border", "curbstone", "stop_line", "virtual"}) {
    std::cout << feature << " : " << parser.getWaysByType(feature).size() << "\n";
  }

  for (const auto & lanelet : lanelets) {
    auto it = lanelet.tags.find("subtype");
    if (it != lanelet.tags.end() && it->second == "road") {
      std::cout << "\nFirst road lanelet: " << lanelet.id << "\n";
      for (const auto & member : lanelet.members) {
        if (member.role == "left" || member.role == "right" || member.role == "centerline") {
          std::cout << "\n " << member.role << "\nCoordinates: [";
          for (std::size_t i = 0; i < member.coordinates.size(); ++i) {
            std::cout << std::setprecision(10)
                      << "(" << member.coordinates[i].lat << ", " << member.coordinates[i].lon << ")";
            if (i + 1 < member.coordinates.size()) {
              std::cout << ", ";
            }
          }
          std::cout << "]\n";
        }
      }
      break;
    }
  }

  return 0;
}
