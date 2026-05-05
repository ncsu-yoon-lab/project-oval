import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wolfwagen/ros2_ws/src/project-oval/project_oval/install/pyvesc'
