# OSM Parser

A Python tool for loading OpenStreetMap (OSM) files and extracting map information.

## Features

- Loads OSM files and reads nodes, ways, and relations.
- Extracts lanelets and their members (left, right, and centerline).
- Converts way node references into latitude/longitude coordinates.
- Extracts map features by type, including curbstones, road borders, and stop lines.

## Setup

Requires Python 3.12 or later.

An OpenStreetMap (OSM) file must be provided when running the parser.

## Usage

Run the parser from the root of the repository, providing the path to an OSM file:

```bash
python3 tools/osm_parser/osm_parser.py path/to/map.osm
```

Import the parser functions into another Python file:

```python
from tools.osm_parser.osm_parser import (
    load_osm,
    get_lanelets,
    get_ways_by_type,
    get_way_coordinates
)
```

### Load an OSM file

```python
nodes, ways, relations, node_dict, way_dict = load_osm("path/to/map.osm")
```

Returns:
- `nodes`, `ways`, `relations`: Lists of OSM elements.
- `node_dict`, `way_dict`: Dictionaries for looking up elements by ID.

### Extract lanelets

```python
lanelets = get_lanelets(relations, way_dict, node_dict)
```

Returns a list of lanelet dictionaries containing their IDs, tags, members, and resolved way coordinates.

Example: Get road lanelet centerlines.

```python
for lanelet in lanelets:
    if lanelet["tags"].get("subtype") == "road":
        for member in lanelet["members"]:
            if member["role"] == "centerline":
                print(member["coordinates"])
```

### Extract map features

```python
curbstones = get_ways_by_type(ways, node_dict, "curbstone")
```

Other feature types that can be extracted include:
- `road_border`
- `line_thin`
- `stop_line`
- `virtual`

### Get way coordinates

```python
coordinates = get_way_coordinates(way_dict["100"], node_dict)
```

Returns an ordered list of `(latitude, longitude)` coordinate pairs.

## How OSM Data Is Organized

- **Nodes:** Individual points with latitude and longitude.
- **Ways:** Ordered sequences of node references.
- **Relations:** Groups of elements, such as lanelets.
- **Members:** Elements referenced by a relation, with a role such as `left`, `right`, or `centerline`.
- **Tags:** Key-value pairs describing an element.

The parser follows this structure to extract lanelet geometry:

`Relation → Members → Ways → Nodes → Coordinates`

## Limitations

- Coordinates remain in latitude/longitude format, they are not converted to local x-y metres.
- Lane connectivity, widths, and lengths are not calculated.
- Regulatory elements are not fully parsed.