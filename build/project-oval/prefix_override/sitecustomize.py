import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/patchy/lab_work/lab_ros_ws/src/project-oval/install/project-oval'
