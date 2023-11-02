Instructions to Start Up Cameras:
  On the left side of the desk with the alienware computer, there is a network switch. 
  Plug in the power cord to the back of it. 
  Then log in to the computer.
  Run the VICON TRACKER application.
  Wait for all the cameras to turn blue, then they are ready to use. 
    Otherwise, if they are red or purple, troubleshoot the cameras.

Run the Vicon node:
  Open a terminal on your car. 
  
    cd ~/ros2_ws2/src/wolfwagen/wolfwagen
    python3 wolfwagen_vicon.py
    
    The position and orientation data will print in the terminal.
    "Car not being picked up by camera, re-run the program, or try to make sure the car is in view of the cameras." will indicate an error.
  
Subscribe to the data:
  You can also have a subscriber in a different file/node.
  Make a callback function and open a subscription in main to the 'vicon_pos'
    def vicon_callback(data):
    // Add your processing logic here
    // "data" has fields for x,y,z, and roll, pitch, yaw, and scalar
    Access the fields as follows:
      data.pose.position.x ( the x-value )
      data.pose.position.y ( the y-value )
      data.pose.position.z ( the z-value )
      data.pose.orientation.x ( the roll-value )
      data.pose.orientation.y ( the pitch-value )
      data.pose.orientation.z ( the yaw-value )
      data.pose.orientation.w ( magnitude of rotation )


    vicon_node = rclpy.create_node('vicon_node')
    subscription = vicon_node.create_subscription(
        PoseStamped,  //  Replace String with the appropriate message type
        'vicon_pos',  //  Specify the topic you want to subscribe to
        vicon_callback  // Specify the callback function
        1  //  Queue size
    )

  Visualize the data in rviz2:
    Run rviz2 in a terminal
    Add a Pose display type
      The display type should show an error at first.
      Open the dropdown and look at the Topic field.
      It should be empty, click into the empty field or hit the dropdown on the right side of the empty field.
      Select vicon_pos.
      Change the dimensions of the Pose arrow if it is too small to see easily.
