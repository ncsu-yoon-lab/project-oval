import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class SidewalkGUI(Node):
    def __init__(self):
        super().__init__('sidewalk_gui')
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_points = None
        
        # Camera intrinsics
        self.fx = 528.12646484375
        self.fy = 528.12646484375
        self.cx = 635.8855590820312
        self.cy = 347.66925048828125

        # Static extrinsic params:
        self.X = -0.25
        self.Y = 0.25
        self.Z = 0.25

        self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, 10)
        self.create_subscription(PointCloud2, '/velodyne_points', self.pointcloud_callback, 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
        

    def pointcloud_callback(self, msg):
        self.latest_points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            # if self.latest_image is not None:
            #     cv2.imshow("ZED Image", self.latest_image)
            if self.latest_points is not None and self.latest_image is not None:
                #black image for visualization
                img = self.latest_image.copy()

                points = np.array(self.latest_points)
                if points.shape[0] == 0:
                    return
            
                # Extract x, y, z from LiDAR
                x = points[:, 0]
                y = points[:, 1]
                z = points[:, 2]
            
                # LiDAR → Camera rotation matrix
                R = np.array([
                    [0, 1,  0],   # X_cam = Y_lidar
                    [0,  0, 1],   # Y_cam = Z_lidar
                    [1,  0,  0]    # Z_cam =  X_lidar
                ])
            
                # Translation from LiDAR to Camera (note the negation)
                t = np.array([ -self.X, -self.Y, -self.Z ])
            
                # Build 4x4 transformation matrix (rotation + translation)
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = t
            
                # Make homogeneous LiDAR point matrix (N x 4)
                lidar_points = np.vstack((x, y, z)).T
                lidar_hom = np.hstack((lidar_points, np.ones((lidar_points.shape[0], 1))))  # (N, 4)
            
                # Transform to camera frame
                cam_points = (T @ lidar_hom.T).T[:, :3]  # (N, 3)
            
                # Filter: only points in front of the camera
                cam_points = cam_points[cam_points[:, 2] > 0]
            
                # Project with intrinsics
                K = np.array([
                    [self.fx, 0, self.cx],
                    [0, self.fy, self.cy],
                    [0, 0, 1]
                ])
                pixels_hom = (K @ cam_points.T).T  # (N, 3)
                pixels = pixels_hom[:, :2] / pixels_hom[:, 2:]
            
                # Flip horizontally and vertically
                pixels[:, 0] = img.shape[1] - pixels[:, 0]  # Horizontal flip
                pixels[:, 1] = img.shape[0] - pixels[:, 1]  # Vertical flip
                
                for pt in pixels:
                    cam_x, cam_y = int(pt[0]), int(pt[1])
                    if 0 <= cam_x < img.shape[1] and 0 <= cam_y < img.shape[0]:
                        cv2.circle(img, (cam_x, cam_y), 2, (0, 255, 0), -1)
            
                cv2.imshow("Velodyne Projection", img)
            key = cv2.waitKey(30)
            if key == 27:
                break
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    gui = SidewalkGUI()
    try:
        gui.run()
    finally:
        gui.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
