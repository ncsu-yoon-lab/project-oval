import json
import math
import httpx
import networkx as nx

from fastapi import FastAPI, HTTPException, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import uvicorn

CERT_FILE = "/etc/gateway/webserver.crt"
KEY_FILE = "/etc/gateway/webserver.key"
CA_FILE = "/etc/gateway/root-ca.crt"

PORT = 8443

GATEWAY_URL = "https://car.local:8443"

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

with open("map.json", "r") as f:
    map_data = json.load(f)

nodes = {node["id"]: node for node in map_data["nodes"]}
edges = map_data["edges"]

G = nx.Graph()

for node in map_data["nodes"]:
    G.add_node(node["id"], x=node["x"], y=node["y"])

for edge in map_data["edges"]:
    a = nodes[edge["from"]]
    b = nodes[edge["to"]]
    weight = math.sqrt((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2)
    G.add_edge(edge["from"], edge["to"], weight=weight)


def heuristic(a, b):
    return math.sqrt((G.nodes[a]["x"] - G.nodes[b]["x"]) ** 2 + (G.nodes[a]["y"] - G.nodes[b]["y"]) ** 2)


@app.get("/")
async def index():
    return FileResponse("static/index.html")


@app.get("/map_data")
async def get_map_data():
    return map_data


@app.post("/find_path")
async def find_path(request: Request):
    body = await request.json()
    start_id = body.get("start")
    goal_id = body.get("goal")

    if not start_id or not goal_id:
        raise HTTPException(status_code=400, detail="Start and goal required")

    try:
        path = nx.astar_path(G, start_id, goal_id, heuristic=heuristic, weight="weight")
    except nx.NetworkXNoPath:
        raise HTTPException(status_code=404, detail="No path found")
    except nx.NodeNotFound:
        raise HTTPException(status_code=400, detail="Invalid node id")

    return {"path": path, "points": [{"x": nodes[n]["x"], "y": nodes[n]["y"]} for n in path]}


@app.post("/send_path")
async def send_path(request: Request):
    points = await request.json()

    if not points:
        raise HTTPException(status_code=400, detail="No points provided")

    async with httpx.AsyncClient(
        cert=(CERT_FILE, KEY_FILE),
        verify=CA_FILE
    ) as client:
        response = await client.post(f"{GATEWAY_URL}/path", json=points)

    if response.status_code != 200:
        raise HTTPException(status_code=502, detail="Gateway rejected the path")

    return {"status": "ok"}


def main():
    uvicorn.run(
        app,
        host="127.0.0.1",
        port=PORT
    )


if __name__ == "__main__":
    main()