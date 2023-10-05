#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import sys
from geometry_msgs.msg import PoseStamped

import ogl_viewer.tracking_viewer as gl
import pyzed.sl as sl

# pose = None
# def poseCallback(data):
#     # # Camera position in map frame
#     # tx = msg.pose.position.x
#     # ty = msg.pose.position.y
#     # tz = msg.pose.position.z

#     # # Orientation quaternion
#     # tf2::Quaternion q(
#     #   msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
#     #   msg->pose.orientation.w);

#     # // 3x3 Rotation matrix from quaternion
#     # tf2::Matrix3x3 m(q);

#     # // Roll Pitch and Yaw from rotation matrix
#     # double roll, pitch, yaw;
#     # m.getRPY(roll, pitch, yaw);

#     # // Output the measure
#     # RCLCPP_INFO(
#     #   get_logger(),
#     #   "Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f - "
#     #   "Ts: %u.%u sec ",
#     #   msg->header.frame_id.c_str(), tx, ty, tz, roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG,
#     #   msg->header.stamp.sec, msg->header.stamp.nanosec);
#     global pose
#     pose = data


# def poseOdomSubscriber(): 
#     node = Node("zed_odom_pose")

#     pose_subscription = node.create_subscription(
#         PoseStamped,
#         "/zed2i/zed_node/pose",
#         pose_callback,
#         20)
if __name__ == "__main__":
    init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
                                 coordinate_units=sl.UNIT.METER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)

    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    # # If applicable, use the SVO given as parameter
    # # Otherwise use ZED live stream
    # if len(sys.argv) == 2:
    #     filepath = sys.argv[1]
    #     print("Using SVO file: {0}".format(filepath))
    #     init_params.set_from_svo_file(filepath)

    zed = sl.Camera()
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    tracking_params = sl.PositionalTrackingParameters()

    tracking_params.enable_pose_smoothing = True
    tracking_params.set_floor_as_origin = False
    tracking_params.enable_area_memory = True
    
    # tracking_params.area_file_path = "wolfwagen.area"
    zed.enable_positional_tracking(tracking_params)


    runtime = sl.RuntimeParameters()
    camera_pose = sl.Pose()

    camera_info = zed.get_camera_information()
    # Create OpenGL viewer
    viewer = gl.GLViewer()
    viewer.init(camera_info.camera_model)

    py_translation = sl.Translation()
    pose_data = sl.Transform()

    text_translation = ""
    text_rotation = ""

    while viewer.is_available():
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            tracking_state = zed.get_position(camera_pose)
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                rotation = camera_pose.get_rotation_vector()
                translation = camera_pose.get_translation(py_translation)
                text_rotation = str((round(rotation[0], 2), round(rotation[1], 2), round(rotation[2], 2)))
                text_translation = str((round(translation.get()[0], 2), round(translation.get()[1], 2), round(translation.get()[2], 2)))
                pose_data = camera_pose.pose_data(sl.Transform())
            viewer.updateData(pose_data, text_translation, text_rotation, tracking_state)
            print( "Tracking state: ", tracking_state )
            print( text_rotation )
            print( text_translation )
            print( pose_data )

    zed.disable_positional_tracking("wolfwagen.area")
    viewer.exit()
    zed.close()
