#!/usr/bin/env python3
import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

class controller:
    def __init__(self):
        rospy.init_node('flight') # 'flight' is name of your ROS node

        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.set_altitude = rospy.ServiceProxy('set_altitude', srv.SetAltitude)
        self.set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
        self.set_yaw_rate = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)

        self.startTelemetry = self.get_telemetry(frame_id='terrain')
        print(self.startTelemetry.x, self.startTelemetry.y, self.startTelemetry.z) 

        self.landed = True

        # Initialize cv_bridge to convert ROS images to OpenCV format.
        self.bridge = CvBridge()

        self.recent_img = None #to store most recent camera image
        #add a camera subscriber and keep track of the most recent image as a class variable
        self.camera_sub = rospy.Subscriber('main_camera/image_raw', Image, self.camera_callback)

        self.range = None #to store the latest range reading
        #Subscribe to the range finder store as a class variable
        self.range_sub = rospy.Subscriber("rangefinder", Range, self.range_callback)
        

    def camera_callback(self, msg):
        #callback function for the camera subscriber and updates the most recent image received
        self.recent_img = msg

    def range_callback(self, msg):
        
        self.range = msg.range

        #Get the topic graph look up rqt graph but there may be other ways than rqt
        #get the transformation tree

    #Function for Take off or Landings
    def takeoff_or_land(self):
        if self.landed == True:
            self.navigate(x=0, y=0, z=2, frame_id='body', auto_arm=True) #takes off to 2m off the ground
            print('Taking off: setting altitude to 3m')
            self.set_altitude(z=3, frame_id='terrain') #sets altitude to 3m off ground
            self.landed = False
        else:
            self.landed = True
            print('Landing')
            self.land()

    def get_next_goal(self, x,y,z):
        print('Navigating to given goal')
        ###TODO make it move to the pose given
        self.navigate(x=x, y=y, z=z, speed=0.5, frame_id='body')
    
    def rotate(self, degrees):
        self.navigate(yaw=math.radians(degrees), frame_id='body')

    def print_pos(self):
        telemetry = self.get_telemetry()
        print(telemetry) 

    def prompt(self):
        i = input("press 1 for takeoff/land, 2 to give a goal, 3 for rotate: ,4 for current position")
        if i == "1": #takeoff or land
            self.takeoff_or_land()

        elif i =="2": #navigate to goal
            pos = input("Input comma seperated numbers for where to go: x,y,z")
            pos = pos.split(",")
            x,y,z = [float(a) for a in pos]
            self.get_next_goal(x, y, z)
            print(f"{x=}, {y=}, {z=}")

        elif i == "3": #rotate drone
            d = int(input("input the degrees to turn: "))
            self.rotate(d)
        
        elif i == "4": #display controller
            self.display_controller()
        else:
            print(f"argument {i} not recognized")

    def display_controller(self):
        """
        Continuously updates an OpenCV window with the latest camera image.
        Overlays the range reading at the top of the image.
        Processes keyboard inputs:
          - W: move forward (increase x)
          - S: move backward (decrease x)
          - A: move left (decrease y)
          - D: move right (increase y)
          - Q: rotate left (yaw positive)
          - E: rotate right (yaw negative)
          - T: take off or land
        """
        print("""Starting display controller. 
          - W: move forward (increase x)
          - S: move backward (decrease x)
          - A: move left (decrease y)
          - D: move right (increase y)
          - Q: rotate left (yaw positive)
          - E: rotate right (yaw negative)
          - T: Take off or land
          - Press ESC to exit the display loop.""")
        while not rospy.is_shutdown():
            # If an image is available, convert it to OpenCV format.
            if self.recent_img is not None:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(self.recent_img, "bgr8")
                except CvBridgeError as e:
                    rospy.logerr(e)
                    continue
            else:
                # Create a blank image if no image is available.
                cv_image = np.zeros((480, 640, 3), np.uint8)

            # Overlay the range reading at the top of the image.
            if self.range is not None:
                text = f"Range: {self.range:.2f} m"
            else:
                text = "Range: N/A"
            cv2.putText(cv_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0), 2, cv2.LINE_AA)

            # Display the image.
            cv2.namedWindow("Drone Camera", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Drone Camera", 1280, 720) #set window to 1280x720 pixels
            cv2.imshow("Drone Camera", cv_image)
            key = cv2.waitKey(30) & 0xFF

            # Process keyboard inputs.
            # Adjust the movement distance or rotation angle as needed.
            if key == ord('w'):
                # Move forward (positive x)
                self.navigate(x=0.5, y=0, z=0, frame_id='body')
            elif key == ord('s'):
                # Move backward (negative x)
                self.navigate(x=-0.5, y=0, z=0, frame_id='body')
            elif key == ord('a'):
                # Move left (negative y)
                self.navigate(x=0, y=0.5, z=0, frame_id='body')
            elif key == ord('d'):
                # Move right (positive y)
                self.navigate(x=0, y=-0.5, z=0, frame_id='body')
            elif key == ord('q'):
                # Rotate counterclockwise by 10 degrees
                self.rotate(10)
            elif key == ord('e'):
                # Rotate clockwise by 10 degrees
                self.rotate(-10)
            elif key == ord('t'):
                # take off or land 
                self.takeoff_or_land()
            elif key == 27:  # ESC key to break out of the loop
                print("Exiting display controller.")
                cv2.destroyAllWindows()
                break
        pass



if __name__ == "__main__":
    c = controller()
    c.display_controller() #immediately launch display only exits with esc key
