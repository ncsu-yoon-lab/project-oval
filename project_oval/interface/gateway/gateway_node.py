import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Everything we need for interacting with a web application
from fastapi import FastAPI, Request, HTTPException
import uvicorn
import threading


app = FastAPI()

PORT = 8080
# certs have readonly perms (400) for ros gateway user
# recently learned about TPMS, Nvidia Orin supports it.
# I think that would be a pretty cool addition. Would really lock down the car security-wise
CERT_FILE = "/etc/gateway/car.crt"
KEY_FILE = "/etc/gateway/car.key"

# Lab root CAR cert. For verification of webserver cert.
CA_FILE = "/etc/gateway/root-ca.crt"

# The gateway node currently, only needs to accept path points.
@app.post("/path")
async def receive_path(points: list):
    if gateway_node is None:
        raise HTTPException(status_code=503, detail="Gateway node not ready for path, please wait.")
    # build the path from points json and publish it.
    path_msg = gateway_node.build_path_msg(points)
    gateway_node.path_pub.publish(path_msg)
    
    return {"status": "success", "path received of length: ": len(points)}


class GatewayNode(Node):
    def __init__(self):
        super().__init__("gateway_node")
        self.path_pub = self.create_publisher(Path, "/pure_pursuit/path", 10)
        self.port = 8080

        # Start a thread for accepting path requests
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()

    def run_server(self):

        uvicorn.run(
            app,
            host="0.0.0.0",
            port=PORT,
            ssl_certfile=CERT_FILE,
            ssl_keyfile=KEY_FILE,
            ssl_ca_certs=CA_FILE, # requires uvicorn check that a cert presented was signed by our root CA.
            ssl_cert_reqs="required" # If you're thinking of changing this, dont.
        )

    def build_path_msg(self, points):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for point in points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(point["x"])
            pose.pose.position.y = float(point["y"])
            pose.pose.position.z = 0.0
            path.poses.append(pose)

        return path

def main(args=None):
    rclpy.init(args=args)
    node = GatewayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

