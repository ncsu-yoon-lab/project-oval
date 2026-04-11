import httpx

# Build a flat list of {x, y} points from path ids and coords.
# coords values are [lat, lon] — gateway reads these as x and y.
def build_payload(path_ids, coords):
    points = []
    for id in path_ids:
        latlon = coords[id]
        points.append({"x": latlon[0], "y": latlon[1]})
    return points
 
 
# POST the path to the car over mTLS.
# Returns a result
# Never raises all errors are captured in the return value.
async def send_path_to_car(path_ids, coords, car_url, cert_file, key_file, ca_file):
    payload = build_payload(path_ids, coords)
 
    try:
        async with httpx.AsyncClient( cert=(cert_file, key_file),verify=ca_file,timeout=3.0,) as client:
            r = await client.post(car_url, json=payload)
        
        # Any 200 should be fine.
        success = 200 <= r.status_code < 300
 
        if success:
            error_msg = None
        else:
            error_msg = "Car returned " + str(r.status_code)
 
        return {
            "ok":     success,
            "status": r.status_code,
            "error":  error_msg,
        }
    except httpx.ConnectError:
        return {"ok": False, "status": None, "error": "Could not reach car — is it online?"}
    except httpx.TimeoutException:
        return {"ok": False, "status": None, "error": "Car connection timed out"}
    except Exception as e:
        print("FULL ERROR:", type(e).__name__, str(e))
        return {"ok": False, "status": None, "error": str(e)}
        