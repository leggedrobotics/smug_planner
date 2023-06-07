# ======================================================================
# Copyright (c) 2023 Changan Chen
# Robotic Systems Lab, ETH Zurich
# All rights reserved.
# This source code is licensed under the MIT license.
# See LICENSE file in the project root for details.
# ======================================================================

import rospy
import copy
import math
import tkinter as tk
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from interactive_markers.menu_handler import *
from smug_planner_msgs.msg import *
import tf


menu_handler = MenuHandler()
server = None
marker_pub = None
marker_array = MarkerArray()
tois_msg = ToIs()

# keep track on current setting to update interactively
n_toi = 0
current_n_poi = 0


def pose_cb(feedback, publisher):
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        box_marker = Marker()
        box_marker.ns = "Start"
        box_marker.id = 0
        box_marker.action = Marker.MODIFY
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 1.07
        box_marker.scale.y = 0.55
        box_marker.scale.z = 0.3
        box_marker.color.r = 0.0
        box_marker.color.g = 1
        box_marker.color.b = 0.0
        box_marker.color.a = 1
        box_marker.pose = feedback.pose
        box_marker.header.frame_id = "map"

        marker_array.markers.append(box_marker)
        marker_pub.publish(marker_array)

        robot_pose = geometry_msgs.msg.PoseStamped()
        robot_pose.header = box_marker.header
        robot_pose.pose = box_marker.pose
        pose_pub.publish(robot_pose)


def update_tentative_toi_cb(feedback):
    p = feedback.pose.position
    print(
        feedback.marker_name
        + " is now at "
        + str(p.x)
        + ", "
        + str(p.y)
        + ", "
        + str(p.z)
    )
    # upadte the current marker if neccesary
    # visualize the toi with a non-interactive marker
    marker = Marker()
    marker.header.frame_id = "map"

    global n_toi

    # add tentative pois
    r = 1
    global current_n_poi

    # get feedback yaw
    (_, _, yaw) = tf.transformations.euler_from_quaternion(
        [
            feedback.pose.orientation.x,
            feedback.pose.orientation.y,
            feedback.pose.orientation.z,
            feedback.pose.orientation.w,
        ]
    )
    for i in range(current_n_poi):
        marker_poi = Marker()
        marker_poi.ns = str(n_toi) + "th toi"
        marker_poi.header.frame_id = "map"
        marker_poi.type = Marker.SPHERE
        marker_poi.action = Marker.MODIFY
        marker_poi.scale.x = 0.2
        marker_poi.scale.y = 0.2
        marker_poi.scale.z = 0.2
        marker_poi.color.r = 0.6
        marker_poi.color.g = 0.6
        marker_poi.color.b = 0.6
        marker_poi.color.a = 0.5
        marker_poi.id = i + 1
        th = i / current_n_poi * 2 * math.pi + yaw
        dx = math.cos(th) * r
        dy = math.sin(th) * r
        marker_poi.pose = copy.deepcopy(feedback.pose)
        marker_poi.pose.position.x += dx
        marker_poi.pose.position.y += dy
        marker_array.markers.append(marker_poi)

    marker_pub.publish(marker_array)


def type_cb(feedback):
    # create a pop-up window
    popup = tk.Tk()
    popup.title("Number of PoI")

    # create a label and an entry widget for the user input
    label = tk.Label(popup, text="Enter the number of poi:")
    label.pack()
    entry = tk.Entry(popup)
    entry.pack()

    # create a function to get the user input
    def get_and_set():
        n_poi = entry.get()  # get the value entered by the user
        print(f"#poi is: {n_poi}")

        # set poi number
        set_n_poi_cb(feedback, int(n_poi))
        popup.destroy()  # close the pop-up window

    # create a button to confirm the input
    button = tk.Button(popup, text="Confirm", command=get_and_set)
    button.pack()

    # set the geometry of the window
    popup.update_idletasks()  # update the window dimensions
    width = popup.winfo_width()
    height = popup.winfo_height()
    x = (popup.winfo_screenwidth() // 2) - (
        width // 2
    )  # center the window horizontally
    y = (popup.winfo_screenheight() // 2) - (
        height // 2
    )  # center the window vertically
    popup.geometry("{}x{}+{}+{}".format(width, height, x, y))

    # run the pop-up window
    popup.mainloop()


def set_n_poi_cb(feedback, n_poi):
    global current_n_poi
    # check if need to delete marker
    if n_poi < current_n_poi:
        # delete redundant markers
        for i in range(current_n_poi - n_poi):
            marker_array.markers[-i - 1].action = Marker.DELETE
        marker_pub.publish(marker_array)

    # update poi number
    current_n_poi = n_poi

    # update the marker immediately
    update_tentative_toi_cb(feedback)
    print("n_poi = ", current_n_poi)


def spawn_toi_cb(feedback):
    # make the current markers a confirmed toi and add them to the msg
    # visualize the toi with a non-interactive marker
    marker = Marker()
    marker.header.frame_id = "map"

    # add toi
    global n_toi
    global current_n_poi
    marker.ns = str(n_toi) + "th toi"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.pose = copy.deepcopy(feedback.pose)
    marker.scale.x = 0.6
    marker.scale.y = 0.6
    marker.scale.z = 0.6
    marker.color.r = 0.6
    marker.color.g = 0.6
    marker.color.b = 0.6
    marker.color.a = 1.0
    marker_array.markers.append(marker)

    # put toi into msg
    toi_msg = TargetOfInterest()
    toi_msg.header.frame_id = "map"
    toi_msg.centroid = marker.pose.position

    # add pois
    contacts = []
    r = 1
    # get feedback yaw
    (_, _, yaw) = tf.transformations.euler_from_quaternion(
        [
            feedback.pose.orientation.x,
            feedback.pose.orientation.y,
            feedback.pose.orientation.z,
            feedback.pose.orientation.w,
        ]
    )
    print("n_poi = ", current_n_poi)
    for i in range(current_n_poi):
        marker_poi = Marker()
        marker_poi.ns = str(n_toi) + "th toi"
        marker_poi.header.frame_id = "map"
        marker_poi.type = Marker.SPHERE
        marker_poi.action = Marker.MODIFY
        marker_poi.scale.x = 0.2
        marker_poi.scale.y = 0.2
        marker_poi.scale.z = 0.2
        marker_poi.color.r = 0.6
        marker_poi.color.g = 0.6
        marker_poi.color.b = 0.6
        marker_poi.color.a = 1.0
        marker_poi.id = i + 1
        th = i / current_n_poi * 2 * math.pi + yaw
        dx = math.cos(th) * r
        dy = math.sin(th) * r
        marker_poi.pose = copy.deepcopy(feedback.pose)
        marker_poi.pose.position.x += dx
        marker_poi.pose.position.y += dy
        marker_array.markers.append(marker_poi)

        # add to msg
        contacts.append(marker_poi.pose.position)

    toi_msg.contacts = contacts
    # add to tois msg
    tois_msg.tois.append(toi_msg)

    marker_pub.publish(marker_array)

    n_toi += 1


def done_cb(feedback, publisher):
    print("mission is set, publishing ... ...")

    global tois_msg
    global marker_array
    # publish the mission
    publisher.publish(tois_msg)

    # clear msg
    # todo:
    tois_msg = ToIs()
    marker_array = MarkerArray()

    # reset global counting
    global current_n_poi
    global n_poi
    n_toi = 0
    current_n_poi = 0


def clear_cb(feedback):
    # reset global counting
    global current_n_poi
    global n_poi
    n_toi = 0
    current_n_poi = 0

    tois_msg = ToIs()
    for marker in marker_array.markers:
        marker.action = Marker.DELETE
    marker_pub.publish(marker_array)


if __name__ == "__main__":
    rospy.init_node("mission_publisher")

    # create a Publisher for the MarkerArray
    marker_pub = rospy.Publisher("mission_markers", MarkerArray, queue_size=1)

    # publisher for the mission
    mission_pub = rospy.Publisher("tois", ToIs, queue_size=1)

    # publisher for the robot pose
    pose_pub = rospy.Publisher(
        "robot_pose", geometry_msgs.msg.PoseStamped, queue_size=1
    )

    # create an interactive marker server for mission set up
    server = InteractiveMarkerServer("toi_marker")

    # create an interactive marker server for robot starting pose
    robot_marker_server = InteractiveMarkerServer("start_pose")

    # create interactive marker for robot pose
    pose_marker = InteractiveMarker()
    pose_marker.header.frame_id = "map"
    pose_marker.name = "start pose"
    pose_marker.description = "Start pose"
    pose_marker.pose.position.x = 1

    # robot bouding box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 1.07
    box_marker.scale.y = 0.55
    box_marker.scale.z = 0.3
    box_marker.color.r = 0.0
    box_marker.color.g = 1
    box_marker.color.b = 0.0
    box_marker.color.a = 0.5

    # robot heading
    head_marker = Marker()
    head_marker.type = Marker.ARROW
    head_marker.scale.x = 0.3
    head_marker.scale.y = 0.1
    head_marker.scale.z = 0.1
    head_marker.pose.position.x = box_marker.pose.position.x + 0.1
    head_marker.color.r = 0.0
    head_marker.color.g = 1
    head_marker.color.b = 0
    head_marker.color.a = 0.5

    # pose menu
    pose_add_control = InteractiveMarkerControl()
    pose_add_control.interaction_mode = InteractiveMarkerControl.BUTTON
    pose_add_control.always_visible = True
    pose_add_control.markers.append(box_marker)
    pose_add_control.markers.append(head_marker)

    # add the control to the interactive marker
    pose_marker.controls.append(pose_add_control)

    # pose control
    pose_control = InteractiveMarkerControl()
    pose_control.orientation.w = 1
    pose_control.orientation.x = 0
    pose_control.orientation.y = 1
    pose_control.orientation.z = 0
    pose_control.name = "move_xy"
    pose_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
    pose_marker.controls.append(copy.deepcopy(pose_control))
    pose_control.name = "move_z"
    pose_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    pose_marker.controls.append(copy.deepcopy(pose_control))

    robot_marker_server.insert(
        pose_marker, lambda feedback, publisher=pose_pub: pose_cb(feedback, publisher)
    )
    robot_marker_server.applyChanges()

    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.name = "toi"
    int_marker.description = "ToI"

    # create a interactive marker
    sp_marker = Marker()
    sp_marker.type = Marker.SPHERE
    sp_marker.scale.x = 0.7
    sp_marker.scale.y = 0.7
    sp_marker.scale.z = 0.7
    sp_marker.color.r = 0.0
    sp_marker.color.g = 1
    sp_marker.color.b = 1
    sp_marker.color.a = 0.5

    # create a button
    menu_control = InteractiveMarkerControl()
    menu_control.interaction_mode = InteractiveMarkerControl.MENU
    menu_control.always_visible = True
    menu_control.markers.append(sp_marker)

    # add the control to the interactive marker
    int_marker.controls.append(menu_control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_xy"
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
    int_marker.controls.append(copy.deepcopy(control))
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(copy.deepcopy(control))

    # add the interactive marker to our collection &
    server.insert(int_marker, update_tentative_toi_cb)

    # create an Interactive Marker Menu
    set_n_poi_entry = menu_handler.insert("Set PoI number")
    confirm_entry = menu_handler.insert("Spawn", callback=spawn_toi_cb)
    menu_handler.insert(
        "Send mission",
        callback=lambda feedback, publisher=mission_pub: done_cb(feedback, publisher),
    )
    menu_handler.insert("Clear mission", callback=clear_cb)

    # give the poi number through typing
    menu_handler.insert("Type PoI number: ", parent=set_n_poi_entry, callback=type_cb)

    # selection of the poi number in the list
    for i in range(15):
        n_poi = i + 1
        menu_handler.insert(
            str(n_poi),
            parent=set_n_poi_entry,
            callback=lambda feedback, n_poi=n_poi: set_n_poi_cb(feedback, n_poi),
        )

    # attach the Menu to the Interactive Marker
    menu_handler.apply(server, int_marker.name)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()
