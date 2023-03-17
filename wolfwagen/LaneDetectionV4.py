import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Int64
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 as cv # OpenCV library
import numpy as np
import time
import math
import random



class LaneDetection(Node):

    def __init__(self):
        super().__init__('lane_detection')
        self.lane = Int64()
        self.lane.data = 0
        #self.lane.speed = 0
        self.subscription = self.create_subscription(
            Image,
            '/zed2i/zed_node/rgb_raw/image_raw_color',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Int64, 'lane' , 10)
        self.br = CvBridge()
        self.subscription  # prevent unused variable warning
        self.timestamp = time.strftime("%H:%M:%S")
        

    def listener_callback(self, msg):
        img = self.br.imgmsg_to_cv2(msg)
        self.process_img(img)
    
    def publish_lane(self):
        #self.throttle = 8171
        self.lane.data = self.direction
        print("THIS IS THE DIRECTION: " , self.direction)
        #self.lane.speed = self.throttle
        self.publisher.publish(self.lane)


    def process_img(self, frame):
        img = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        print(img.shape)
        height, width = img.shape
        img = img[360:height-1, :]

        _, img = cv.threshold(img, 150, 180, cv.THRESH_BINARY)
        
        #dark_yellow = np.array([30,100,100])
        #light_yellow = np.array([62,255,255])

        #preparing the mask to overlay
        #mask = cv.inRange(img, dark_yellow, light_yellow)

        #The black region in the mask has the value of 0,
        #so when multiplied with original image removes all non-yellow regions
        result = img

        edge = cv.Canny(result,100,500) # you can adjust these min/max values
        lines = cv.HoughLinesP(edge, rho = 1, theta = np.pi/180, threshold = 100 , minLineLength = 20, maxLineGap = 150)

        horizontal_midpoint = img.shape[1]/2
        cte = 0 # Cross Track Error
        cnt_left = 0    # number of left lines
        cnt_right = 0   # number of right lines
        
        if lines is None: 
            #print("No lines detected")
            cv.waitKey(1) 
        else:

            line_img = np.zeros((img.shape[0],img.shape[1],3),dtype=np.uint8)
            line_color=[0, 255, 0]
            line_thickness=1
            dot_color = [61, 217, 108]
            dot_size = 25

            left_line_x = []    #x-values of left lines 
            left_line_y = []    #y-values of left lines 
            right_line_x = []   #x-values of right lines 
            right_line_y = []   #x-values of right lines
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if x2 - x1 == 0:
                        continue
                    slope = (y2 - y1) / float(x2 - x1)
                    #===============================
                    # filter out irrelevant lines
                    if abs(slope)<0.5:
                        continue
                    line_len = np.linalg.norm(np.array((x1, y1)) - np.array((x2, y2)))
                    if line_len < 20:
                        continue
                    #===============================
                
                    #draw lines (only relevant ones)
                    #cv.line(line_img, (x1, y1), (x2, y2), line_color, line_thickness)
                    #cv.circle(line_img, (x1, y1), dot_size, dot_color, -1)
                    #cv.circle(line_img, (x2, y2), dot_size, dot_color, -1)
                    
                    #===============================

                    if x1 < horizontal_midpoint and x2 < horizontal_midpoint:
                        if slope > 0: continue
                        left_line_x.extend((x1, x2))
                        left_line_y.extend((y1, y2))
                        cnt_left += 1
                    elif x1 > horizontal_midpoint and x2 > horizontal_midpoint:
                        if slope < 0: continue
                        right_line_x.extend((x1, x2))
                        right_line_y.extend((y1, y2))
                        cnt_right += 1
                    #===============================
                    #cv.imshow('lines', line_img)

            #print(str(distance_left) + " to the left"+ "\n" + str(distance_right) + " to the right" )
            time.sleep(0.1) #adjust this to make the camera look at different rates
            #print("New set of lines:")
            #print(lines)
            pos_slope = []
            neg_slope = []
            for i in lines:
                x1, y1, x2, y2= i[0]
                if ((y2 - y1)/(x2 - x1)) > 0:
                    pos_slope.append(i[0])
                else:
                    neg_slope.append(i[0])

            try:
                # Positive Lines
                plines_x1 = []
                plines_x2 = []
                plines_y1= []
                plines_y2 = []
                for pos in pos_slope:
                    x1, y1, x2, y2= pos
                    plines_x1.append(x1)
                    plines_x2.append(x2)
                    plines_y1.append(y1)
                    plines_y2.append(y2)
                px1 = int(sum(plines_x1)/len(plines_x1))
                px2 = int(sum(plines_x2)/len(plines_x2))
                py1 = int(sum(plines_y1)/len(plines_y1))
                py2 = int(sum(plines_y2)/len(plines_y2))
                
                # Negative Lines
                nlines_x1 = []
                nlines_x2 = []
                nlines_y1= []
                nlines_y2 = []
                for neg in neg_slope:
                    x1, y1, x2, y2= neg
                    nlines_x1.append(x1)
                    nlines_x2.append(x2)
                    nlines_y1.append(y1)
                    nlines_y2.append(y2)
                

                nx1 = int(sum(nlines_x1)/len(nlines_x1))
                nx2 = int(sum(nlines_x2)/len(nlines_x2))
                ny1 = int(sum(nlines_y1)/len(nlines_y1))
                ny2 = int(sum(nlines_y2)/len(nlines_y2))

                # average line
                ax1 = int((px1 + nx2)/2)
                ax2 = int((px2 + nx1)/2)
                ay1 = int((py1 + ny2)/2)
                ay2 = int((py2 + ny1)/2)
                # print("pos",pos_slope)
                # print("neg",neg_slope)
                # print(px1, py1, px2, py2)
                # print(nx1, ny1, nx2, ny2)
                # print(ax1, ay1, ax2, ay2)


                #positive lines
                cv.line(result, (px1, py1) , (px2, py2) , (67, 31, 115) , 3)
                #cv.circle(result, (nx1, ny1), dot_size, dot_color, -1)
                #cv.circle(result, (x2, y2), dot_size, dot_color, -1)
                
                #negative lines
                cv.line(result, (nx1, ny1) , (nx2, ny2) , (0,0,255) , 3)
                #average lines
                cv.line(result, (ax1 , ay1) , (ax2 , ay2) , (255,0,0) , 3)

                dleft = nx1 - 700
                dright = 700 - px2

                if (dleft > dright):
                   #print("Too far to the right")
                   self.direction = abs(dleft)
                else:
                   self.direction = dright
                print("dleft: " , dleft)
                print("dright: " , dright)
                print(self.direction)
                self.publish_lane()
                #self.get_logger().info('Publishing: "%s"' % self.lane)
            except ZeroDivisionError:
                pass
            
        cv.imshow('frame', frame)
        #cv.imshow('mask', mask)
        cv.imshow('result', result)

        cv.waitKey(1) 





def main(args=None):
    rclpy.init(args=args)

    lane_detection = LaneDetection()

    rclpy.spin(lane_detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lane_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
