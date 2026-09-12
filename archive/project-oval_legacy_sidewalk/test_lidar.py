import open3d as o3d
import numpy as np
import sys
import os
from natsort import natsorted
from tqdm import tqdm
import time

def preprocessing(pcd):
    points = np.asarray(pcd.points)
    # Calculate angle in XY plane
    angles = np.arctan2(points[:,1], points[:,0])  # radians
    # Keep points between -90 and +90 degrees (i.e., in front)
    angle_mask = (angles >= -np.pi/2) & (angles <= np.pi/2)

    # Remove points higher than 30 cm
    heights = points[:, 2]
    height_mask = (heights <= 1.0)

    # Remove points further than 10 m
    dist = points[:, 0]
    distance_mask = (dist <= 10.0)

    # Combine all masks
    mask = angle_mask & height_mask & distance_mask
    filtered_points = points[mask]

    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

    if pcd.has_colors():
        colors = np.asarray(pcd.colors)[mask]
        filtered_pcd.colors = o3d.utility.Vector3dVector(colors)
    if pcd.has_normals():
        normals = np.asarray(pcd.normals)[mask]
        filtered_pcd.normals = o3d.utility.Vector3dVector(normals)
    
    return filtered_pcd

# Get the walls 
def predictor(pcd):
    points = np.asarray(pcd.points)
    
    y_points = points[:, 1]
    left_mask = (y_points < 0)  # Left wall
    right_mask = (y_points > 0)  # Right wall

    left_points = points[left_mask]
    right_points = points[right_mask]

    if len(left_points) == 0 or len(right_points) == 0:
        print("No points detected on one or both walls.")
        return 0.0
    
    left_pcd = o3d.geometry.PointCloud()
    left_pcd.points = o3d.utility.Vector3dVector(left_points)
    right_pcd = o3d.geometry.PointCloud()
    right_pcd.points = o3d.utility.Vector3dVector(right_points)

    # Fit a plane to the left wall
    left_plane_model, left_inliers = left_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    # Fit a plane to the right wall
    right_plane_model, right_inliers = right_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

    a_left, b_left, c_left, d_left = left_plane_model
    a_right, b_right, c_right, d_right = right_plane_model

    # Check if the planes are vertical
    normal_left = np.array([a_left, b_left, c_left])
    normal_left = normal_left / np.linalg.norm(normal_left)
    normal_right = np.array([a_right, b_right, c_right])
    normal_right = normal_right / np.linalg.norm(normal_right)

    # Check if normal_left is close to horizontal (z axis)
    if np.abs(normal_left[2]) > 0.1 or np.abs(normal_right[2]) > 0.1:
        print("Detected planes are not vertical enough to be walls.")
        return 0.0
    else:
        # Calculate distance to left and right planes
        left_distance = np.abs(d_left) / np.linalg.norm(normal_left)
        right_distance = np.abs(d_right) / np.linalg.norm(normal_right)

    # Give steering angle based on the distance to the walls
    mean_width = (left_distance + right_distance) / 2.0
    steering_angle = np.arctan2(right_distance - left_distance, 1.0)  # 1.0 is a placeholder for the distance to the front
    
    print(f"Mean width of planes: {mean_width:.2f} m, Steering angle: {np.degrees(steering_angle):.2f} degrees")

    return mean_width

# Get the walls 
def get_ground_plane(pcd):
    points = np.asarray(pcd.points)

    z_points = points[:, 2]
    ground_mask = (z_points <= np.percentile(z_points, 10))  # Lowest 10% of z-values
    ground_points = points[ground_mask]  # Assuming ground points have z < 0.1
    
    # print(f"Number of ground points: {len(ground_points)}")
    
    # if len(ground_points) == 0:
        # print("No ground points found.")

    # Fit a plane to the ground points
    if len(ground_points) < 3:
        # print("Not enough ground points to fit a plane.")
        return None
    
    ground_pcd = o3d.geometry.PointCloud()
    ground_pcd.points = o3d.utility.Vector3dVector(ground_points)
    # Use RANSAC to fit a plane
    plane_model, inliers = ground_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    a, b, c, d = plane_model
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)
    # Check if the normal is close to vertical (z axis)
    if np.abs(normal[2]) < 0.9:
        # print("Detected plane is not horizontal enough to be the ground.")
        return None
    # print(f"Ground plane equation: {a}x + {b}y + {c}z + {d} = 0")
    return plane_model

def create_transparent_plane_mesh(plane_model, width=3.0, height=5.0):
    a, b, c, d = plane_model
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)
    center = -d * normal

    # Create a square in the XY plane (z=0)
    mesh = o3d.geometry.TriangleMesh.create_box(width=width, height=height, depth=0.01)
    mesh.translate(-mesh.get_center())

    # Align the box (plane) normal to the detected plane normal
    z_axis = np.array([0, 0, 1])
    axis = np.cross(z_axis, normal)
    angle = np.arccos(np.clip(np.dot(z_axis, normal), -1.0, 1.0))
    if np.linalg.norm(axis) > 1e-6:
        axis = axis / np.linalg.norm(axis)
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
        mesh.rotate(R, center=(0, 0, 0))

    mesh.translate(center)
    mesh.paint_uniform_color([0.855, 0.97, 0.88])  # green
    return mesh

    
def main():
    if len(sys.argv) < 2:
        print("Usage: python lidar_nav.py <input dir>")
        return
    pcd_path = sys.argv[1]

    pcd_files = natsorted(os.listdir(pcd_path))
    pcd_files = pcd_files[750:1000]  # Limit to first 100 files for testing
    print(f"Found {len(pcd_files)} PCD files in {pcd_path}")

    valid_plane_count = 0

    # vis = o3d.visualization.Visualizer()
    # vis.create_window(window_name="PCD Sequence", width=1920, height=1080)

    # view_ctl = vis.get_view_control()

    # camera_params_saved = None  # Placeholder

    for pcd_file in pcd_files:
        pcd_file_path = os.path.join(pcd_path, pcd_file)
        pcd = o3d.io.read_point_cloud(pcd_file_path)

        # Data preprocessing
        filtered_pcd = preprocessing(pcd)

        # Get width of hallway
        mean_width = predictor(filtered_pcd)

        # get steering angle
        # plane_model = get_ground_plane(filtered_pcd)
        
        # Build current frame geometries
        # geometries = [filtered_pcd]
        # if plane_model is not None:
        #     mesh = create_transparent_plane_mesh(plane_model, height=mean_width)
        #     geometries.append(mesh)
        #     valid_plane_count += 1
    
    #     # Clear previous geometries (without resetting the camera view)
    #     vis.clear_geometries()
    
    #     # Add new geometries for this frame
    #     for g in geometries:
    #         vis.add_geometry(g)

    #     # Restore user-defined view after the first frame
    #     if camera_params_saved:
    #         view_ctl.convert_from_pinhole_camera_parameters(camera_params_saved)
    
    #     vis.poll_events()
    #     vis.update_renderer()
    #     time.sleep(0.1)

        
    #     camera_params_saved = view_ctl.convert_to_pinhole_camera_parameters()
    
    # vis.destroy_window()

    print(f"Number of pcd files with ground planes detected: {valid_plane_count}")
    print("Processing complete.")
    

if __name__ == "__main__":
    main()