from pathlib import Path

# Server
PORT = 8443
HOST = "0.0.0.0"

# Data
DATA_FILE = Path("oval_points.json")

# Car endpoint
CAR_URL = "https://oval.car:8080/path"
TELEOP_URL = "https://oval.car:8080/cmd_vel"

# mTLS certs
CERT_FILE = str(Path("~/pki/webserver/webserver.crt").expanduser())
KEY_FILE  = str(Path("~/pki/webserver/webserver.key").expanduser())
CA_FILE   = str(Path("~/pki/root/root-ca.crt").expanduser())