from pathlib import Path
import xml.etree.ElementTree as ET
import argparse


# Load an OSM file and prepare its elements for parsing

def load_osm(filename):

    file_path = Path(filename)

    if not file_path.is_file():
        raise FileNotFoundError(
            f"OSM file not found: {file_path}"
        )

    tree = ET.parse(file_path)
    root = tree.getroot()

    nodes = root.findall("node")
    ways = root.findall("way")
    relations = root.findall("relation")

    # Create dictionaries for looking up elements by ID
    node_dict = {node.get("id"): node for node in nodes}
    way_dict = {way.get("id"): way for way in ways}

    return nodes, ways, relations, node_dict, way_dict

# Convert an OSM way into a list of coordinates
def get_way_coordinates(way, node_dict):

    coordinates = []

    for nd in way.findall("nd"):

        node_id = nd.get("ref")
        node = node_dict[node_id]

        coordinates.append((
            float(node.get("lat")),
            float(node.get("lon"))
        ))

    return coordinates


# Extract all lanelet relations
def get_lanelets(relations, way_dict, node_dict): 

    lanelets = []

    for relation in relations:

        tags = {
            tag.get("k"): tag.get("v")
            for tag in relation.findall("tag")
        }

        if tags.get("type") == "lanelet":

            members = []

            for member in relation.findall("member"):

                member_type = member.get("type")
                member_id = member.get("ref")
                role = member.get("role")

                member_data = {
                    "type": member_type,
                    "id": member_id,
                    "role": role
                }

                # Resolve way members into their coordinates
                if member_type == "way" and member_id in way_dict:

                    way = way_dict[member_id]

                    member_data["tags"] = {
                        tag.get("k"): tag.get("v")
                        for tag in way.findall("tag")
                    }

                    member_data["coordinates"] = get_way_coordinates(
                        way, node_dict
                    )

                members.append(member_data)

            lanelets.append({
                "id": relation.get("id"),
                "tags": tags,
                "members": members
            })

    return lanelets


# Extract ways belonging to a particular feature type
def get_ways_by_type(ways, node_dict, feature_type):

    results = []

    for way in ways:

        tags = {
            tag.get("k"): tag.get("v")
            for tag in way.findall("tag")
        }

        if tags.get("type") == feature_type:

            results.append({
                "id": way.get("id"),
                "type": feature_type,
                "tags": tags,
                "coordinates": get_way_coordinates(way, node_dict)
            })

    return results


# Run the parser

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Parse an OpenStreetMap (OSM) file."
    )

    parser.add_argument(
        "filename",
        help="Path to the OSM file to parse."
    )

    args = parser.parse_args()

    nodes, ways, relations, node_dict, way_dict = load_osm(args.filename)

    print("OSM file loaded successfully!")
    print("Nodes:", len(nodes))
    print("Ways:", len(ways))
    print("Relations:", len(relations))

    # Extract lanelets
    lanelets = get_lanelets(relations, way_dict, node_dict)

    print("\nTotal lanelets:", len(lanelets))

    # Extract different feature types
    for feature_type in [
        "line_thin",
        "road_border",
        "curbstone",
        "stop_line",
        "virtual"
    ]:

        features = get_ways_by_type(ways, node_dict, feature_type)

        print(feature_type, ":", len(features))

    # Example: inspect the first road lanelet
    first_road_lanelet = next(
        (
            lanelet for lanelet in lanelets
            if lanelet["tags"].get("subtype") == "road"
        ),
        None
    )

    if first_road_lanelet is not None:

        print("\nFirst road lanelet:", first_road_lanelet["id"])

        for member in first_road_lanelet["members"]:

            if member["role"] in ["left", "right", "centerline"]:

                print("\n", member["role"].upper())
                print("Coordinates:", member["coordinates"])
