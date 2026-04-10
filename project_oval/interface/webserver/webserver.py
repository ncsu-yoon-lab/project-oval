# fast api imports
from fastapi import FastAPI, HTTPException, Request

from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
# uvicorn
import uvicorn

import config
from lib.graph import load_data, build_graph


# Startup config files.

DATA         = load_data(config.DATA_FILE)
COORDS, ADJ  = build_graph(DATA)
VALID_IDS    = set(COORDS.keys())

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")


# Routes

@app.get("/")
async def index():
    return FileResponse("static/index.html")

# get fraph for frontend
@app.get("/graph")
async def graph():
    # should be fine, comes from config file on car.
    adj_serialized = {}
    for k, neighbors in ADJ.items():
        adj_serialized[str(k)] = {}
        for neighbor_id, weight in neighbors.items():
            adj_serialized[str(k)][str(neighbor_id)] = weight

    return {
        "origin": DATA["origin"],
        "nodes":  DATA["nodes"],
        "points": DATA["points"],
        "adj":    adj_serialized,
    }


@app.post("/find_path")
async def find_path(request):
    body  = await request.json()
    start = body.get("start")
    goal  = body.get("goal")

    if start is None or goal is None:
        raise HTTPException(status_code=400, detail="start and goal required")
    if start not in VALID_IDS or goal not in VALID_IDS:
        raise HTTPException(status_code=400, detail="Invalid node id")
    if start == goal:
        raise HTTPException(status_code=400, detail="Start and goal must differ")

    path_ids = astar(start, goal, COORDS, ADJ)
    if not path_ids:
        raise HTTPException(status_code=404, detail=f"No path from {start} to {goal}")

    # Build path from astar
    coords = []
    for i in path_ids:
        coords.append({"id": i, "latlon": COORDS[i]})

    return {
        "path":       path_ids,
        "coords":     coords,
        "distance_m": round(path_distance(path_ids, COORDS), 1),
    }


@app.post("/send_path")
async def send_path(request: Request):
    body     = await request.json()
    path_ids = body.get("path", [])

    if len(path_ids) < 2:
        raise HTTPException(status_code=400, detail="Path must have at least 2 points")

    result = await send_path_to_car(
        path_ids,
        COORDS,
        config.CAR_URL,
        config.CERT_FILE,
        config.KEY_FILE,
        config.CA_FILE,
    )

    if not result["ok"]:
        raise HTTPException(status_code=502, detail=result["error"])

    return {"status": "ok", "car_status": result["status"]}


# Entry point.

if __name__ == "__main__":
    uvicorn.run(app, host=config.HOST, port=config.PORT)