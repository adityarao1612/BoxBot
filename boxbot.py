import cv2
import numpy as np
import math


# Niryo2 setup:
from niryo_robot_python_ros_wrapper import *
import rospy
import time

#print("Do you want to get 1.blue 2.red or 3.green item?")
#choice=int(input())

choice=2

hsv_lower=[100, 50, 50]
hsv_upper=[200, 255, 255]

if choice==3:
    hsv_lower=[40, 50, 50]
    hsv_upper=[80, 255, 255]

rospy.init_node('niryo_ned_example_python_ros_wrapper')

niryo_robot = NiryoRosWrapper()
workspace_name = "conveyorkejriwal"  # Robot's Workspace Name
niryo_robot.calibrate_auto()
niryo_robot.update_tool()

pick_pose = (0, -0.27, 0.25, 3.14, 1.35, 1.57)
drop_pose = (0.228, 0, 0.20, 3.14, 1.57, 1.57)


def find_object_midpoint(image, hsv_lower, hsv_upper, min_area, max_area):

    global reference_midpoint
    # Convert the image from BGR to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper HSV values for the specific object
    lower_bound = np.array(hsv_lower, dtype=np.uint8)
    upper_bound = np.array(hsv_upper, dtype=np.uint8)

    # Create a binary mask for the specific HSV value
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    # Find contours in the binary mask
    _, contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Calculate the midpoint of each contour
    midpoints = []
    for contour in contours:

        area = cv2.contourArea(contour)
        if min_area < area < max_area:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                midpoints.append((cX, cY))
                print("midpoint of item=", cX, cY)

                cv2.drawMarker(image, (cX, cY), (0, 0, 255),
                               markerType=cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=2)
                cv2.drawContours(image, [contour], 0, (0, 255, 0), 2)

    return (cX-320), image, (cY-220)

def find_red_object_midpoint(image, min_area, max_area):

    global reference_midpoint
    # Convert the image from BGR to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

    # Combine the two masks to get the full red range
    mask = cv2.bitwise_or(mask_red1, mask_red2)

    # Find contours in the binary mask
    _, contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Calculate the midpoint of each contour
    midpoints = []
    for contour in contours:

        area = cv2.contourArea(contour)
        if min_area < area < max_area:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                midpoints.append((cX, cY))

                print("midpoint of item=", cX, cY)

                cv2.drawMarker(image, (cX, cY), (0, 0, 255),
                               markerType=cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=2)
                cv2.drawContours(image, [contour], 0, (0, 255, 0), 2)

   # cv2.imshow("Image with Midpoints", image)
    return (cX-(image.shape[1])/2), image, (cY-(image.shape[0])/2)



def getImageFromNiryo():
    img = niryo_robot.get_compressed_image()
    mtx, dist = niryo_robot.get_camera_intrinsics()

    fo = open("niryo_cam.jpeg", "w")
    fo.write(img)
    fo.close()

    dist_img = cv2.imread("niryo_cam.jpeg", -1)

    matrix = [mtx[i:i+3] for i in range(0, len(mtx), 3)]
    undist_img = cv2.undistort(dist_img, np.array(matrix), dist)

    cv2.imwrite("img2.jpg", undist_img)
   # return 


# MOVE TO PICK POSE
niryo_robot.move_pose(*pick_pose)
in_center = False



#First position the camera to align to center of required object in y-axis

while in_center == False:
    # Take screenshot of niryo camera
    getImageFromNiryo()

    undist_img = cv2.imread("img2.jpg", -1)

    if choice==2:
        _, midpoint, pixel_to_move, = find_red_object_midpoint(
            undist_img, 19000*0, 24000*100)
    
    else:
        _, midpoint, pixel_to_move, = find_object_midpoint(
            undist_img,hsv_lower, hsv_upper, 19000*0, 24000*100)
    #cv2.imshow("Image with Midpoint  s", midpoint)
    if (abs(pixel_to_move) < 50):
        in_center = True
    if pixel_to_move < 0:
        niryo_robot.jog_pose_shift([0,
                                   min(pixel_to_move*0.0001, -0.005), 0, 0, 0, 0])
    else:
        niryo_robot.jog_pose_shift([0,
                                   max(pixel_to_move*0.0001, +0.005), 0, 0, 0, 0])

    time.sleep(0.1)

in_center = False

#After aligning in Y-axis, align in X-axis
while in_center == False:

    # Take screenshot of niryo camera
    getImageFromNiryo()

    undist_img = cv2.imread("img2.jpg", -1)

    if choice==2:
        pixel_to_move, midpoint, _ = find_red_object_midpoint(
            undist_img, 19000*0, 24000*100)
    
    else:
        _, midpoint, pixel_to_move, = find_object_midpoint(
            undist_img,hsv_lower, hsv_upper, 19000*0, 24000*100)
    #cv2.imshow("Image with Midpoint  s", midpoint)
    if (abs(pixel_to_move) < 10):
        in_center = True

    if pixel_to_move > 0:
        niryo_robot.jog_pose_shift(
            [min(pixel_to_move*0.0001, -0.002), 0, 0, 0, 0, 0])
    else:
        niryo_robot.jog_pose_shift(
            [max(pixel_to_move*0.0001, +0.002), 0, 0, 0, 0, 0])

    time.sleep(0.1)

cv2.destroyAllWindows()

# pick up object:--

niryo_robot.release_with_tool()
currentpose = niryo_robot.get_pose_as_list()
j = niryo_robot.get_joints()

    #Move effector to the camera frame position
niryo_robot.move_pose(currentpose[0]+math.cos(j[0])/14, currentpose[1]+math.sin(j[0])/17,
                      currentpose[2],  3.14, 1.57, 1.57)

currentpose = niryo_robot.get_pose_as_list()
niryo_robot.move_pose(currentpose[0], currentpose[1],
                      0.108,  3.14, 1.57, 1.57)

niryo_robot.grasp_with_tool()


#Dropping object:---
currentpose = niryo_robot.get_pose_as_list()
niryo_robot.move_pose(currentpose[0], currentpose[1], 0.3,  3.14, 1.57, 1.57)

niryo_robot.move_pose(*drop_pose)


niryo_robot.open_gripper()
niryo_robot.release_with_tool()


niryo_robot.move_pose(*pick_pose)
