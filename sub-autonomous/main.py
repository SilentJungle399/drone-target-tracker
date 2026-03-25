from logger_config import setup_logging

setup_logging("sub_autonomous")

from config import ANGLE_DEG, SPACING

from controller import MavlinkController
from mission import Mission
from vision import Vision

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from pyproj import Transformer
from shapely.affinity import rotate, translate
from shapely.geometry import LineString, MultiLineString, Polygon


def read_kml_polygon(kml_path: Path) -> Polygon:
    kml_xml = kml_path.read_text(encoding="utf-8")
    root = ET.fromstring(kml_xml)
    ns = {"kml": "http://www.opengis.net/kml/2.2"}

    coord_elem = root.find(".//kml:Polygon//kml:coordinates", ns)
    if coord_elem is None:
        raise ValueError("No Polygon found in KML file")

    coords = []
    for token in coord_elem.text.strip().split():
        lon, lat, *_ = map(float, token.split(","))
        coords.append((lon, lat))

    if len(coords) < 3:
        raise ValueError("Polygon must have at least 3 points")

    if coords[0] != coords[-1]:
        coords.append(coords[0])

    return Polygon(coords)


def generate_lawnmower(poly: Polygon, spacing_m: float, angle_deg: float, safety_buffer_m: float = 4.0):
    to_m = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    to_ll = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)

    poly_m = Polygon([to_m.transform(x, y) for x, y in poly.exterior.coords])

    poly_m = poly_m.buffer(-safety_buffer_m)
    if poly_m.is_empty:
        raise ValueError("Polygon collapsed after safety buffer")

    minx, miny, maxx, maxy = poly_m.bounds
    cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
    diag = math.hypot(maxx - minx, maxy - miny)

    lines = []
    y = -diag
    while y <= diag:
        lines.append(LineString([(-diag, y), (diag, y)]))
        y += spacing_m

    lines = [rotate(line, angle_deg, origin=(0, 0)) for line in lines]
    lines = [translate(line, xoff=cx, yoff=cy) for line in lines]

    clipped = []
    for line in lines:
        inter = poly_m.intersection(line)
        if inter.is_empty:
            continue
        if isinstance(inter, LineString):
            clipped.append(inter)
        elif isinstance(inter, MultiLineString):
            clipped.extend(inter.geoms)

    return clipped, to_ll


def generate_waypoints(lines, transformer: Transformer, angle_deg: float):
    def order_key(line: LineString):
        c = line.centroid
        theta = math.radians(angle_deg + 90)
        return c.x * math.cos(theta) + c.y * math.sin(theta)

    lines = [line for line in lines if not line.is_empty]
    lines.sort(key=order_key)

    waypoints = []
    for i, line in enumerate(lines):
        pts = list(line.coords)

        if i % 2 == 1:
            pts.reverse()

        for x, y in pts:
            lon, lat = transformer.transform(x, y)
            waypoints.append([round(lat, 9), round(lon, 9)])

    return waypoints

def get_kml_waypoints(kml_path: Path):
    kml_path = Path("map.kml")

    if not kml_path.exists():
        raise FileNotFoundError(f"KML file not found: {kml_path}")

    polygon = read_kml_polygon(kml_path)
    lines, transformer = generate_lawnmower(
        polygon,
        spacing_m= SPACING,
        angle_deg= ANGLE_DEG,
    )
    waypoints = generate_waypoints(lines, transformer, angle_deg = ANGLE_DEG)

    print(f"Generated {len(waypoints)} waypoints.")
    
    return waypoints[::-1]

def main():
    # waypoints = get_kml_waypoints(Path("map.kml"))

	# lhc ground
    waypoints = [
        [29.9451178, 76.8172203],
        [29.9450621, 76.8173772],
		[29.9448785, 76.8173812],
	]

    controller = MavlinkController()
    vision = Vision()

    try:
        mission = Mission(
            controller, 
            waypoints = waypoints, 
            vision = vision
        )
        mission.run()
    finally:
        pass
        vision.release()


if __name__ == "__main__":
    main()
