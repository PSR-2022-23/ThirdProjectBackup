#!/usr/bin/env python3

#Mostly chatgpt code and rviz documentation code, done with lack of sleep, probably will crash thread carefuly
import os
import rospy
import json
from functools import partial
import math
import random
import time
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tkinter as tk
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


# Global variables
server = None
pose = None
bridge = CvBridge()
images = {"camera": None, "object": None, "yolo": None}
labels = []

def positionCallback(msg):
    global pose
    pose = msg.pose.pose

def labelCallback(msg):
    label_str = msg.data
    global labels
    labels = label_str.split("\n")

def yoloCallback(msg):
    global images
  
    try:
        images["yolo"] = bridge.imgmsg_to_cv2(msg, "8UC3")
    except CvBridgeError as e:
        print('Failed to convert image:', e)
        return

def get_json_file_contents(file_path):
    try:
        with open(file_path, 'r') as json_file:
            contents = json.load(json_file)
        return contents
    except json.JSONDecodeError:
        raise Exception("Invalid JSON file")

#^generic functions

#make marker and text

def make_menu_marker():

    # ChatGPT cube marker
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    marker.color.r = marker.color.g = marker.color.b = 0.5
    marker.color.a = 0.1
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.scale = 1
    int_marker.name = "menu_marker"
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True
    control.markers.append(marker)
    int_marker.controls.append(control)
    server.insert(int_marker)
    server.applyChanges()


def make_text_marker(text = "Unknown", color = [0.5, 0.5, 0.5]):
    marker = Marker()
    marker.type = Marker.TEXT_VIEW_FACING
    marker.text = text
    marker.scale.z = 0.37
    marker.pose.position.z = 0.75
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "text_marker"
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True
    control.markers.append(marker)
    int_marker.controls.append(control)
    global server
    if server is None:
        server = InteractiveMarkerServer("menu")
    server.insert(int_marker)
    server.applyChanges()



def goal_done_callback(state, result):

    if state == 3:
        print("Goal reached")
        make_text_marker( text = "Goal reached",color = [0.5, 0.5, 0.5])
    else:
        print("Goal canceled")

#goals make and stop
def give_goal( _ , goal, goal_dict, wait=False):

    if goal in goal_dict:
        goal_coordinates = goal_dict[goal]
        goal_x = float(goal_coordinates["x"])
        goal_y = float(goal_coordinates["y"])
        goal_r = float(goal_coordinates["r"])

        # Move the robot
        print("Moving to goal: {}".format(goal))
        move_to_position(goal_x, goal_y, goal_r, wait)
        make_text_marker(text="Moving to \"{}\"".format(goal), color=[0.3, 0.8, 0.3])
    else:
        print("Invalid goal: {}".format(goal))
        make_text_marker(text="Invalid goal", color=[0.8, 0.2, 0.2])



def stop_rob_callback(_, linear, angular, text):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.cancel_all_goals()

    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular

    end_time = rospy.get_rostime().secs + 0.5
    while rospy.get_rostime().secs < end_time:
        publisher.publish(twist)

    make_text_marker(text=text, color=[0.2, 0.2, 0.8])

#move man
def move_to_position(x, y, r, wait=False):
    print("Moving to ({}, {}, {})".format(x, y, r))
    
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(x)
    goal.target_pose.pose.position.y = float(y)
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = float(r)
    goal.target_pose.pose.orientation.w = 1.0
    
    client.send_goal(goal, done_cb=goal_done_callback)
    
    if wait:
        client.wait_for_result()


#send direcly to xy
def coordinates_callback( _ ):

    root = tk.Tk()
    root.title("Enter Coordinates")
    root.withdraw()

    x_label = tk.Label(root, text="X:")
    x_label.grid(row=0, column=0, padx=12, pady=12)

    x_entry = tk.Entry(root)
    x_entry.grid(row=0, column=1, padx=12, pady=12)

    y_label = tk.Label(root, text="Y:")
    y_label.grid(row=1, column=0, padx=12, pady=12)

    y_entry = tk.Entry(root)
    y_entry.grid(row=1, column=1, padx=12, pady=12)

    r_label = tk.Label(root, text="R (rad):")
    r_label.grid(row=2, column=0, padx=12, pady=12)

    r_entry = tk.Entry(root)
    r_entry.grid(row=2, column=1, padx=12, pady=12)
    #nested function for the 4th time in my life pog
    def submit():
        x_raw = x_entry.get()
        y_raw = y_entry.get()
        r_raw = r_entry.get()
        try:
            x = float(x_raw)
            y = float(y_raw)
            r = float(r_raw)

            if -10 < x < 10 and -10 < y < 10:
                valid_input = True
            else:
                valid_input = False
        except ValueError:
            valid_input = False

        if not valid_input:
            make_text_marker(text=f"Invalid input!", color=[0.8, 0.2, 0.2])
            return

        make_text_marker(text=f"Moving to...\n{round(x,1)} / {round(y,1)} / {round(r,1)} rad", color=[0.3, 0.8, 0.3])
        move_to_position(x, y, r)

        root.withdraw()

    submit_button = tk.Button(root, text="Submit", command=submit)
    submit_button.grid(row=3, column=1, pady=10)

    root.deiconify()
    root.mainloop()


#find models probably doenst work
def search_object(model_name):
    global images
    global labels
    success = False
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    twist = Twist()
    twist.linear.x = 0

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.cancel_all_goals()

    make_text_marker( text = "Looking for \"{}\"".format(model_name),color = [0.2, 0.2, 0.8])

    rotation_speed = 0.3
    time_end = rospy.get_rostime().secs + 2*math.pi/rotation_speed

    while rospy.get_rostime().secs < time_end:
        rospy.sleep(0.5)
        twist.angular.z = rotation_speed
        publisher.publish(twist)

        # Check yolo labels
        if model_name in ["bowl", "ball", "person"]:
            images["object"] = images["yolo"]
            success = True
            break
    twist.angular.z = 0
    publisher.publish(twist)

    make_text_marker(text="Object found!" if success else "Object not found!", color=[0.2, 0.8, 0.2] if success else [0.8, 0.2, 0.2])
    return success


def search_callback( _ , model_name, location, goal_dict = {}):

    success = False
    if isinstance(location, str):
        make_text_marker( text = "Looking for \"{}\" in \"{}\"".format(model_name, location),color = [0.2, 0.8, 0.2])
        give_goal(0, goal=location, goal_dict=goal_dict, wait=True)
        success = search_object(model_name)

    # Here
    elif location == 0:
        success = search_object(model_name)
        return success

    # Everywhere
    elif location == 1:
        self_x = float(pose.position.x)
        self_y = float(pose.position.y)
        goal_list = list(goal_dict)
        goal_name = goal_list[0]   
        min_distance = 1000000000000000000

        for goal in goal_dict.keys():
            goal_x = float(goal_dict[goal]["x"])
            goal_y = float(goal_dict[goal]["y"])
            distance = math.sqrt((goal_x-self_x)**2+(goal_y-self_y)**2)

            if distance < min_distance:
                min_distance = distance
                goal_name = goal

        index = goal_list.index(goal_name)
        length = len(goal_list)
        goal_list *= 2
        new_goals = goal_list[index:index+length]

        for goal_name in new_goals:
            make_text_marker( text = "Looking for \"{}\" in \"{}\"".format(model_name, goal_name),color = [0.2, 0.8, 0.2])
            give_goal(0, goal=goal_name, goal_dict=goal_dict, wait=True)
            success = search_object(model_name)
            if success:
                return success

    if not success:
        make_text_marker( text = "Object not found!",color = [0.8, 0.2, 0.2])
        return success
#VC_labs
   
def imageCallback(msg):
    global images
    try:
        images["camera"] = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print('Failed to convert image:', e)
        return
    if images["camera"] is not None:
        cv2.imshow("Robutler's POV", images["camera"])
        cv2.waitKey(1)
    if images["object"] is not None:
        cv2.imshow("Robutler's POV", images["object"])
        images["object"] = None
        cv2.waitKey(3000)



def take_picture_callback( _ ):
    path_this = os.path.realpath(os.path.dirname(__file__))
    time_str = time.strftime("%Y-%m-%d_%H-%M-%S")
    full_path = path_this + "/photos/" + time_str + ".png"
    success = cv2.imwrite(full_path, images["camera"])
    print("Saved") if success else   print("Failed to save image")



def initMenu(menu_handler):
    goal_dict = get_json_file_contents(os.path.realpath(os.path.dirname(__file__))+ "/knownlocations.json")
    object_list = ["bowl", "ball", "person"]
 
    # Move to... coordinates/saved_location
    move_tab = menu_handler.insert( "Move to" )
    menu_handler.insert("Go to coordinates...", parent=move_tab, callback=coordinates_callback)

    for goal in goal_dict.keys():
        menu_handler.insert(goal, parent=move_tab, callback=partial(give_goal, goal = goal, goal_dict = goal_dict))
    menu_handler.insert( "Stop", callback=partial(stop_rob_callback, linear=0, angular=0, text="Stopped"))

    # Look for Object...
    search_tab = menu_handler.insert( "Look for Object..." )
    for object_name in object_list:

        search_tab_2 = menu_handler.insert(object_name, parent=search_tab)
        menu_handler.insert("Here", parent=search_tab_2, callback=partial(search_callback, model_name = object_name, location = 0))
        search_tab_3 = menu_handler.insert("Room...", parent=search_tab_2)
    
        for goal in goal_dict.keys():
            menu_handler.insert(goal, parent=search_tab_3, callback=partial(search_callback, model_name = object_name, location = goal, goal_dict = goal_dict))

        menu_handler.insert("Everywhere", parent=search_tab_2, callback=partial(search_callback, model_name = object_name, location = 1, goal_dict = goal_dict))
    menu_handler.insert( "Smile!", callback=take_picture_callback)


#call funcs
def main():

    # Initialize
    rospy.init_node("menu")
    global server
    server = InteractiveMarkerServer("menu")
    menu_handler = MenuHandler()

    # Create menu
    initMenu(menu_handler)
    make_menu_marker()
    menu_handler.apply(server, "menu_marker")
    make_text_marker(text = "Ready")
    server.applyChanges()

    rospy.Subscriber('/odom', Odometry, positionCallback)
    rospy.Subscriber("/camera/rgb/image_raw", Image, imageCallback)
    rospy.Subscriber("/yolov7/yolov7_label", String, labelCallback)
    rospy.Subscriber("/yolov7/yolov7/visualization", Image, yoloCallback)

    while not rospy.is_shutdown():
        rospy.sleep(10)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()