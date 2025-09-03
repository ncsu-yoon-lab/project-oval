import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import threading
import open3d as o3d

steering_angle = 0.0
lidar_points = None

min_points = 300  # Minimum number of points in a plane to consider it valid
distance_threshold = 0.05 # Distance threshold for RANSAC plane segmentation
ransac_n = 3 # Number of points to sample for RANSAC
num_iterations = 1000

def is_parallel_to_xz(normal):
    "Get the normal vector of the plane and check if it is parallel to the XZ plane"
    normal = normal / np.linalg.norm(normal)
    return np.abs(normal[1]) > 0.9  # y-component is dominant

def distance_to_plane(plane_model):
    "Calculate the distance from the origin to the plane defined by the plane model"
    a, b, c, d = plane_model
    # Distance from origin (0, 0, 0) to the plane ax + by + cz + d = 0
    return np.abs(d) / np.sqrt(a**2 + b**2 + c**2)

def get_lidar_points(msg):
    "Lidar callback to process incoming PointCloud2 messages"
    global lidar_points
    lidar_points = np.array(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))

def preprocessing(points):
    "Remove points outside the field of view and filter based on height and distance"
    angles = np.arctan2(points[:, 1], points[:, 0])
    
    # FOV of 180 degrees (±90 degrees)
    # Filter points within the FOV
    angle_mask = (angles >= -np.pi/2) & (angles <= np.pi/2)

    # Remove points higher than 30cm
    heights = points[:, 2]
    height_mask = (heights <= 0.3)

    # Remove points further than 7m
    dist = points[:, 0]
    dist_mask = (dist <= 7.0)
    filtered_points = points[angle_mask & height_mask & dist_mask]
    
    return filtered_points

def get_wall_planes(pcd):
    " Get the left and right wall planes for robot navigation"
    left_planes = []
    right_planes = []
    left_wall = None
    right_wall = None

    # Segment the largest plane
    while True:
        plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations)
        
        if len(inliers) < min_points:
            break

        # Plane equation: ax + by + cz + d = 0
        [a,b,c,d] = plane_model
        normal = np.array([a, b, c])

        # Only handle planes that are parallel to the XZ plane
        if is_parallel_to_xz(normal):
            # Find the centroid of the inlier points
            inlier_cloud = pcd.select_by_index(inliers)
            centroid = np.mean(np.asarray(inlier_cloud.points), axis=0)

            # Classify as left/right based on y position
            if centroid[1] > 0:
                left_planes.append(plane_model)
            else:
                right_planes.append(plane_model)

        # Remove inliers from pcd for next iteration
        pcd = pcd.select_by_index(inliers, invert=True)

    # Sort planes by distance to the origin
    if len(left_planes):
        left_planes = sorted(left_planes, key=distance_to_plane)
        left_wall = left_planes[0]
    
    if len(right_planes):
        right_planes = sorted(right_planes, key=distance_to_plane)
        right_wall = right_planes[0]

    return left_wall, right_wall

def visualize_2d_walls(plane_model):
    "Visualize given plane in 2D"
    
    a, b, _, d = plane_model

    ## Create some points for the line
    x_vals = np.linspace(-3, 3, 100)
    y_vals = -(a * x_vals + d) / b  # Rearranging ax + by + d = 0 to get y

    # Represent like points_flattened
    line_points = np.vstack([x_vals, y_vals, np.zeros_like(x_vals)]).T

    line_points_pcd = o3d.geometry.PointCloud()
    line_points_pcd.points = o3d.utility.Vector3dVector(line_points)
    line_points_pcd.paint_uniform_color([1, 0, 0])  # Red color for the line

    return line_points_pcd

def visualize_2d(pcd):
    "Visualize the point cloud in 2D by flattening it to the XY plane"
    points = np.asarray(pcd.points)
    points_xy = points[:, :2]  # Take only x and y coordinates

    points_flattened = np.hstack([points_xy, np.zeros((points_xy.shape[0], 1))])

    points_flattened_pcd = o3d.geometry.PointCloud()
    points_flattened_pcd.points = o3d.utility.Vector3dVector(points_flattened)

    return points_flattened_pcd

def predict_steering_angle(left_plane, right_plane):
    "Predict the steering angle based on the left and right wall planes"
    global steering_angle  # Default: go straight

    # Ego line along x-axis (y=0.05)
    ego_y = 0.05
    ego_x = np.linspace(-3, 3, 100)
    ego_line_points = np.vstack([ego_x, ego_y * np.ones_like(ego_x), np.zeros_like(ego_x)]).T

    # Check distance from ego line to each plane
    def point_to_plane_dist(point, plane):
        a, b, c, d = plane
        x, y, z = point
        return np.abs(a*x + b*y + c*z + d) / np.sqrt(a**2 + b**2 + c**2)

    threshold = 0.15  # meters, threshold for "close"

    if left_plane is not None:
        dists = [point_to_plane_dist(pt, left_plane) for pt in ego_line_points]
        if np.min(dists) < threshold:
            steering_angle = 5.0  # steer right (away from left wall)

    if right_plane is not None:
        dists = [point_to_plane_dist(pt, right_plane) for pt in ego_line_points]
        if np.min(dists) < threshold:
            steering_angle = -5.0  # steer left (away from right wall)

    # Apply steering angle to ego line (rotate about origin)
    # c, s = np.cos(steering_angle), np.sin(steering_angle)
    # R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    # ego_line_points_rot = ego_line_points @ R.T

    ego_line_pcd = o3d.geometry.PointCloud()
    ego_line_pcd.points = o3d.utility.Vector3dVector(ego_line_points)
    ego_line_pcd.paint_uniform_color([0, 0, 1])  # Blue color for the ego line

    return ego_line_pcd

def main(args=None):
    rclpy.init(args=args)
    node = Node('lidar_navigator')

    node.create_subscription(PointCloud2, '/velodyne_points', get_lidar_points, 10)
    publisher = node.create_publisher(Float64, '/lidar/predicted_steering_angle', 10)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    ## Open3D visualization setup
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="PCD Sequence", width=1920, height=1080)
    view_ctl = vis.get_view_control()
    camera_params_saved = None  # Placeholder
    
    rate = node.create_rate(10)  # 10 Hz
    while rclpy.ok():
        if lidar_points is not None:
            # Preprocess the LiDAR points
            vis.clear_geometries()
            filtered_points = preprocessing(lidar_points)


            # Store in pcd format to use open3d segment plane
            filtered_pcd = o3d.geometry.PointCloud()
            filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

            ## Visualize the filtered points in 2D
            points_flattened_pcd = visualize_2d(filtered_pcd)
            geometries = [points_flattened_pcd]

            # Segment the left and right wall planes
            left_wall, right_wall = get_wall_planes(filtered_pcd)

            ## 2D visualization of the point cloud and plane
            if left_wall is not None:
                left_wall_2d_pcd = visualize_2d_walls(left_wall)
                geometries.append(left_wall_2d_pcd)

            if right_wall is not None:
                right_wall_2d_pcd = visualize_2d_walls(right_wall)
                geometries.append(right_wall_2d_pcd)

            ## Draw ego-line and predict steering angle
            ego_line = predict_steering_angle(left_wall, right_wall)
            geometries.append(ego_line)

            # # Add new geometries for this frame
            for g in geometries:
                vis.add_geometry(g)

            # Publish the predicted steering angle
            msg = Float64()
            msg.data = steering_angle
            publisher.publish(msg)

            ## Restore user-defined view after the first frame
            if camera_params_saved:
                view_ctl.convert_from_pinhole_camera_parameters(camera_params_saved)

            ## Viz stuff
            vis.poll_events()
            vis.update_renderer()
            camera_params_saved = view_ctl.convert_to_pinhole_camera_parameters()
        
        else:
            node.get_logger().warn("No LiDAR points received yet.")

        rate.sleep()

    
    vis.destroy_window()
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()