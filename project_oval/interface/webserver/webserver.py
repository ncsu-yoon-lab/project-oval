#!/usr/bin/env python3
import json
import math
import heapq
from pathlib import Path

import httpx
from fastapi import FastAPI, HTTPException, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import uvicorn

# --- Config ---
PORT        = 8443
DATA_FILE   = Path("oval_points.json")
CAR_URL     = "https://car.local:8443/path"
CERT_FILE   = "/etc/gateway/webserver.crt"
KEY_FILE    = "/etc/gateway/webserver.key"
CA_FILE     = "/etc/gateway/root-ca.crt"

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")



# 

@app.get("/")
async def index():
    return FileResponse("static/index.html")



@app.post("/find_path")
async def find_path(request: Request):
    body = await request.json()
    start = body.get("start")
    goal  = body.get("goal")

    if start is None or goal is None:
        raise HTTPException(status_code=400, detail="start and goal required")
    if start not in VALID_IDS or goal not in VALID_IDS:
        raise HTTPException(status_code=400, detail="Invalid node id")
    if start == goal:
        raise HTTPException(status_code=400, detail="Start and goal must differ")

    path_ids = astar(start, goal)
    if not path_ids:
        raise HTTPException(status_code=404, detail=f"No path from {start} to {goal}")

    path_coords = [{"id": i, "latlon": COORDS[i]} for i in path_ids]
    total_dist  = sum(
        haversine(COORDS[path_ids[i]], COORDS[path_ids[i + 1]])
        for i in range(len(path_ids) - 1)
    )

    return {
        "path":       path_ids,
        "coords":     path_coords,
        "distance_m": round(total_dist, 1),
    }


@app.post("/send_path")
async def send_path(request: Request):
    body = await request.json()
    path_ids = body.get("path", [])
    if len(path_ids) < 2:
        raise HTTPException(status_code=400, detail="Path must have at least 2 points")

    payload = {
        "start":     {"id": path_ids[0],  "latlon": COORDS[path_ids[0]]},
        "waypoints": [{"id": i, "latlon": COORDS[i]} for i in path_ids[1:-1]],
        "finish":    {"id": path_ids[-1], "latlon": COORDS[path_ids[-1]]},
    }

    try:
        async with httpx.AsyncClient(
            cert=(CERT_FILE, KEY_FILE),
            verify=CA_FILE,
            timeout=3.0,
        ) as client:
            r = await client.post(CAR_URL, json=payload)
        if r.status_code != 200:
            raise HTTPException(status_code=502, detail=f"Car rejected path: {r.status_code}")
        return {"status": "ok", "car_status": r.status_code}

    except httpx.ConnectError:
        raise HTTPException(status_code=503, detail="Could not reach car — is it online?")
    except httpx.TimeoutException:
        raise HTTPException(status_code=504, detail="Car connection timed out")


# Entry point.

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=PORT)