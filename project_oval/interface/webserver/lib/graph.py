
import json
import math
from pathlib import Path
 
 
# Load map JSON.
def load_data(path):
    raw = path.read_text(encoding="utf-8").strip()
    try:
        return json.loads(raw)
    except json.JSONDecodeError as e:
        print(e)
        inner = json.loads(raw)
        return json.loads(inner)
 
 
# Great circle distance in meters between two lat lon points.
def haversine(a, b):
    R = 6_371_000.0
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * R * math.asin(math.sqrt(h))
 
 
# Build coordinate and adjacency maps from raw map data.
# coords id (int) -> [lat, lon]
# adj    id (int) -> {neighbor_id (int) -> distance (float, meters)}
def build_graph(data):
    coords = {}
    adj    = {}
    for n in data["nodes"]:
        coords[n["id"]] = n["latlon"]
    for p in data["points"]:
        coords[p["id"]] = p["latlon"]
 
    for vid in coords:
        adj[vid] = {}
 
    for p in data["points"]:
        pid = p["id"]
        for qid in p["linked_points"]:
            if qid not in coords:
                continue
            w = haversine(coords[pid], coords[qid])
            if qid not in adj[pid] or w < adj[pid][qid]:
                adj[pid][qid] = w
                adj[qid][pid] = w
    return coords, adj
 