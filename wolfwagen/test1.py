import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import Int64, Int64MultiArray
import threading

turn_val = None
test_val = 0
inc_val = True

solution = None

def ld_signal_callback(data):
	global solution
	solution = data.data

def process_img(val):
     global inc_val
     if val % 60 == 0:
          return True
     else:
          return False

def turn_processing():
    global solution, inc_val

    if solution is not None:
        print(len(solution))
    
    return -1

def main(args=None):
    global test_val, solution
    rclpy.init(args=args)
    node = Node("Lane_detection_node")

		
    sub_ld_turnval = node.create_subscription(Int64MultiArray, 'ld_turn_val', ld_signal_callback, 1)
    pub_ld_turnsig = node.create_publisher(Int64, 'ld_turn_sig', 1)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 20	
    rate = node.create_rate(FREQ, node.get_clock())

	
    turning = False	#Am i making a turn (left or right)?
    turning_direction = 0	#1: left, 2: right
    got_solution = False

    while rclpy.ok():
        if not got_solution:
            data = Int64()
            data.data = int(1)
            pub_ld_turnsig.publish(data)

            if solution is not None:
                got_solution = True
        else:
            data = Int64()
            data.data = int(0)
            pub_ld_turnsig.publish(data)
            intersection_bool = process_img(test_val)
            if intersection_bool:
                turning_direction = turn_processing()

        rate.sleep()

	
	# Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
