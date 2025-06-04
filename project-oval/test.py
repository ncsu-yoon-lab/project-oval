import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import os

try:
    from sensor_msgs_py import point_cloud2
except ImportError:
    raise ImportError("Please install sensor_msgs_py for this script.")

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.listener_callback,
            10)
        self.i = 0  # Counter for filenames
        self.save_dir = 'pcd_test'
        os.makedirs(self.save_dir, exist_ok=True)

    def write_pcd(self, filename, points):
        # points: list of (x, y, z, intensity)
        with open(filename, 'w') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z intensity\n')
            f.write('SIZE 4 4 4 4\n')
            f.write('TYPE F F F F\n')
            f.write('COUNT 1 1 1 1\n')
            f.write(f'WIDTH {len(points)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points)}\n')
            f.write('DATA ascii\n')
            for pt in points:
                f.write(f"{pt[0]} {pt[1]} {pt[2]} {pt[3]}\n")

    def listener_callback(self, msg):
        try:
            points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
            if not points:
                self.get_logger().warn('Received empty point cloud.')
                return

            filename = os.path.join(self.save_dir, f'pointcloud_{self.i}.pcd')
            self.write_pcd(filename, points)
            self.get_logger().info(f'Saved point cloud to: {filename}')
            self.i += 1
        except Exception as e:
            self.get_logger().error(f'Error saving point cloud: {e}')

def main(args=None):
    rclpy.init(args=args)
    pointcloud_saver = PointCloudSaver()
    rclpy.spin(pointcloud_saver)
    pointcloud_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()