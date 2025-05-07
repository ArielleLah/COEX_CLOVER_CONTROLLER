#!/usr/bin/ python
import os, sys


HERE = os.path.dirname(os.path.abspath(__file__))

# Insert the src/ directory (where smolagents actually lives)
SRCROOT = os.path.join(HERE, "local_smolagents", "src")
if SRCROOT not in sys.path:
    sys.path.insert(0, SRCROOT)

from smolagents import tool, CodeAgent, OpenAIServerModel# Import CodeAgent and HfApiModel from the agents module
OpenAIServerModel.flatten_messages_as_text = True
OpenAIServerModel.kwargs = {}

import rospy, time
import json
import requests
from typing import List, Dict
from controller import drone_controller # Import your drone controller from the coex_clover_controller package
import numpy as np
from clover import srv
from key import API_KEY
from PIL import Image
import io
import base64
from openai import OpenAI
import cv2
# Instantiate the drone controller.
drone_ctrl = drone_controller()

@tool
def takeoff_or_land() -> bool:
    """
    Commands the drone to either take off or land.
    
    If the drone's internal 'landed' flag is True, this function will attempt to take off.
    If the flag is False, it will attempt to land.
    
    Returns:
        bool: True if the desired state change (takeoff or landing) is successfully achieved.
    """
    if drone_ctrl.landed:
        print("Drone is landed. Initiating takeoff...")
        drone_ctrl.takeoff_or_land()  # Command to take off
        # Optionally, wait a bit to allow takeoff
        rospy.sleep(10)
        # Update and check state if necessary (or rely on the controller's flag)
        if not drone_ctrl.landed:
            print("Takeoff successful.")
            return True
        else:
            print("Takeoff failed.")
            return False
    else:
        print("Drone is in flight. Initiating landing...")
        drone_ctrl.takeoff_or_land()  # Command to land
        rospy.sleep(12)
        if drone_ctrl.landed:
            print("Landing successful.")
            return True
        else:
            print("Landing failed.")
            return False

@tool
def is_grounded() ->bool:
    """
    This tool checks if the drone is in flight or not.
    
    Returns:
        bool: True when drone is on the ground.
    """
    return drone_ctrl.landed

@tool
def get_telemetry()->srv.GetTelemetry:
    """
    This tool checks the current robot telemetry.
    
    Returns:
        srv.GetTelemetry: with the following attributes:

    frame_id (string): current frame (ex: frame="body")
    mode (string) : current flight mode
    x (float), y, (float) z (float): local position of the drone (m)
    lat (float64), lon (float64) : drone latitude and longitude (degrees), requires GPS module;
    alt (float32) : altitude in the global coordinate system (according to WGS-84 standard, not AMSL!), requires GPS module;
    vx (float32), vy(float32), vz(float32) : drone velocity in each x, y, z directions (m/s);
    roll (float32): roll angle (radians);
    pitch (float32): pitch angle (radians);
    yaw (float32): yaw angle (radians);
    roll_rate (float32): angular roll velocity (rad/s);
    pitch_rate (float32): angular pitch velocity (rad/s);
    yaw_rate (float32): angular yaw velocity (rad/s);
    voltage (float32): total battery voltage (V);
    cell_voltage (float32): battery cell voltage (V).
    """
    telemetry = drone_ctrl.get_telemetry()
    #print(telemetry.yaw)
    #print(f"{type(telemetry)=}")
    #print(f"{dir(telemetry)=}")
    return telemetry


@tool
def flight_command(x: float, y: float, z: float, degrees: float, timeout: float = 10) -> bool:
    """
    This tool commands the drone to navigate using the given parameters:
        x: forward/backward position (meters),
        y: left/right position (meters),
        z: altitude change (meters),
        degrees: rotation (clockwise in degrees).
        timeout: float for how many seconds to try to attempt to reach the goal
    This is only to be used when in the air.
    
    Returns:
        bool: True if the difference in distance given to the starting distance is less than 0.3.

    Args:
        x (float): move the drone forward (positive number) or backward (negative number)
        y (float): move the drone left (positive number) or right (negative number)
        z (float): move the drone up (positive number) or down (negative number)
        degrees (float): rotate the drone counterclockwise (positive number) or clockwise (negative number)
        timeout (flaot): how many seconds to try to attempt to reach the goal
    """
    if timeout is None:
        timeout = 10
    
    startTelemetry = drone_ctrl.get_telemetry()
    target_x = startTelemetry.x + x
    target_y = startTelemetry.y + y
    target_z = startTelemetry.z + z
    print('target x: ',target_x, 'target y: ', target_y, 'target_z: ', target_z)
    drone_ctrl.go_to_goal(x, y, z, degrees)
    #rospy.sleep(12) #wait 10 seconds for simulation to run
    #endTelemetry = drone_ctrl.get_telemetry()
    start_time = time.time()
    while time.time() - start_time < timeout:
        telemetry = drone_ctrl.get_telemetry()
        x_squared_diff = (telemetry.x-target_x)**2
        y_squared_diff = (telemetry.y-target_y)**2
        z_squared_diff = (telemetry.z-target_z)**2
        #can implement rotational difference
        if np.sqrt(x_squared_diff + y_squared_diff + z_squared_diff) < 0.3:
            return True
        rospy.sleep(0.1)
    return False

def encode_image(img_array):
    """
    Encodes image as JPEG for use with OPENAI api
    """
    if img_array is None:
        #print(f"image array is none")
        return None
    #print(f"{type(img_array)=}")
    #print(f"{dir(img_array)=}")

    # Convert the ndarray to a PIL Image
    image = Image.fromarray(img_array)
    
    # Create a BytesIO object to save the image
    buffered = io.BytesIO()
    image.save(buffered, format="JPEG")  # Specify the format you want
    buffered.seek(0) #Possibly not needed
    # Get the byte data and encode to base64
    encoded_string = base64.b64encode(buffered.read()).decode('utf-8')
    
    return encoded_string

@tool
def detect_object(target_str: str) -> bool:
    """
    This tool detects a target object by using the most recent image from the drone camera.  
    Compares the target object with the most recent image to find the target.
    
    Returns:
        bool: True if most recent image is showing the target object

    Args:
        target_str (String): A string describing the target object to look for. target_str = user_input.
    """
    print("target_str: ",target_str)
    encoded_img = None
    while encoded_img is None:
        if drone_ctrl.recent_img is not None: #shows last image processed
            cv2.imshow("Last_searched_Img", drone_ctrl.recent_img)
            cv2.waitKey(1)
        encoded_img = encode_image(drone_ctrl.recent_img)
    img_type = "image/jpeg"
    img_query = f"do you see {target_str} in the image response, answer with EXACTLY True or False"

    client = OpenAI(
        api_key= API_KEY,
    )
    gpt_response = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "system",
         "content": [{"type": "text","text": "You are an assistant that inspects images and tells me if the object,{target_str}, appears in them."}]},

        {"role": "user",
         "content": [
                {"type": "text", "text": img_query},
                {"type": "image_url", "image_url": {"url": f"data:{img_type};base64,{encoded_img}"}}]}],      

    response_format={"type": "text"},    
    temperature=0
    )
    output_text = gpt_response.choices[0].message.content.strip()
    print("GPT replied:", output_text)
    return output_text == "True"

    

class Agent():
    def __init__(self):
        # Initialize the ROS node.
        #rospy.init_node('smol_drone_agent', anonymous=False) can only call once, trying to keep controller.py "flight" node
    
        #model used
        m = OpenAIServerModel(
            model_id="gpt-4o-mini",
            api_base="https://api.openai.com/v1",
            api_key=API_KEY,
        )


        self.agent = CodeAgent(
            tools=[takeoff_or_land, flight_command, is_grounded, get_telemetry, detect_object], 
            model=m, 
            add_base_tools=True)

    def run(self, prompt):
        """
        Main loop: waits for terminal input and processes it.
        """
        self.agent.run(prompt)
        

if __name__ == '__main__':
    a = Agent()
    print("Agent ready. Type commands, or 'quit' to exit.")
    try:
        while True:  # Continuous prompt loop :contentReference[oaicite:10]{index=10}
            user_input = input("Enter an object to find or enter quit to quit: ")
            if user_input.strip().lower() == 'quit':
                print("Exiting program…")  # Clean shutdown message :contentReference[oaicite:11]{index=11}
                break  # Terminate the loop and end the script :contentReference[oaicite:12]{index=12}
            # Otherwise, pass to your agent:
            try:
                agent_prompt =f"""
                    You are an autonomous drone‑pilot agent.  
                    Your tools:
                    • takeoff_or_land(): toggles between landed and airborne.  
                    • is_grounded(): returns True if on the ground, False if airborne.  
                    • flight_command(x, y, z, degrees): moves by x/y/z meters (relative to body frame) and rotates by degrees.  
                    • get_telemetry(): returns class with attributes like the x, y, z coordinates
                    • detect_object(target_str) → bool: returns True if the camera image matches the target object, target_str = {user_input}

                    Mission: search a 10 m × 10 m box, in 1 m increments, using detect_object check if object is within the image. Never change altitude after takeoff.
                    
                    1. **Take off & establish altitude**  
                    - Call `takeoff_or_land()` 
                    2. **Define bounds**  
                    - x_min = 0, x_max = 10  
                    - y_min = 0, y_max = 10  
                    - Absolute constraint: never allow x or y outside 0…12.  
                    3. Grid sweep algorithm  
                        1. After takeoff, call get_telemetry() once to get (x0, y0, z0); keep z = z0 forever.  
                        2. Define your search box:  
                            • x_min = 0, x_max = 12  
                            • y_min = 0, y_max = 12  
                            • Always enforce 0 ≤ x,y ≤ 12 before each move.  
                        3. Initialize direction dx = +1.  
                        4. For row_index from 0 up to 12 (inclusive):  
                                a. **Traverse exactly 12 steps in X**  
                                – Repeat 12 times:  
                                    • Call flight_command(dx, 0, 0, 0)  
                                    • Call get_telemetry() to read (x,y)  
                                    • Call detect_object(target_str = {user_input}); if returns True, pull coordinates with get_telemetry and report "{user_input} found at (x,y)" & stop
                                b. **If this was the last row (row_index == 12), break out.**  
                                c. **Move one row over in Y**  
                                • Call flight_command(0, +1, 0, 0)  
                                • Call get_telemetry() to read (x,y)  
                                d. **Reverse direction**  
                                – Set dx = –dx  
                        5. If you finish all 13 rows without finding the target, report “Target not found.”
                    4. **Boundary enforcement**  
                    - Before _any_ `flight_command`, check `get_telemetry()`; if the next x or y would fall outside 0…12, reverse direction or skip that move.  
                    5. Remember where you have searched
                """
                response = a.run(agent_prompt)
                print("Agent response:", response)
            except Exception as e:
                print("Error during agent run:", e)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        if rospy.core.is_initialized():
            rospy.signal_shutdown("User requested shutdown")
        sys.exit(0)


