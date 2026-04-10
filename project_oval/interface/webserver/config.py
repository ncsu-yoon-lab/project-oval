from pathlib import Path

# Server
PORT = 8443
HOST = "127.0.0.1"

# Data
DATA_FILE = Path("oval_points.json")

# Car endpoint
CAR_URL = "https://car.local:8443/path"

# mTLS certs
CERT_FILE= "/etc/gateway/webserver.crt"
KEY_FILE = "/etc/gateway/webserver.key"
CA_FILE = "/etc/gateway/root-ca.crt"