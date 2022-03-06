#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf
from tf.transformations import quaternion_from_euler

from lab5_pkg.srv import captureResponse, capture

import cv2  # Import OpenCV modules

# **** Globals
# Camera frame -> the frame that the point cloud is attached to
camera_frame = "/camera_rgb_optical_frame"
# Baxter's frame
robot_frame = "/base"
# Status flags that sets whether to capture an image or point cloud
qImage = False
qDepth = False
# Image and point cloud
rgb_image = None
pointcloud = None

# Turn this True if you want to see intermidiate results
show_images = False


def transformToBase(pt3D):
    # Transforms a point from the cameras optical frame to the base of the robot
    global camera_frame
    global robot_frame
    q = quaternion_from_euler(0, 0, 0)
    # TF transform
    listener = tf.TransformListener()
    listener.waitForTransform(camera_frame, robot_frame, rospy.Time(), rospy.Duration(1.0))
    camera_point = PoseStamped()
    camera_point.header.frame_id = camera_frame
    camera_point.header.stamp = rospy.Time()
    camera_point.pose.position.x = pt3D[0]
    camera_point.pose.position.y = pt3D[1]
    camera_point.pose.position.z = pt3D[2]
    camera_point.pose.orientation.x = q[0]
    camera_point.pose.orientation.y = q[1]
    camera_point.pose.orientation.z = q[2]
    camera_point.pose.orientation.w = q[3]
    base_point = listener.transformPose(robot_frame, camera_point)

    base_point.pose.orientation.x = 0  # q[0]
    base_point.pose.orientation.y = 0  # q[1]
    base_point.pose.orientation.z = 0  # q[2]
    base_point.pose.orientation.w = 1  # q[3]
    return base_point


def readImage(msg):
    # Callback for image cloud topic
    global qImage
    global rgb_image

    if qImage:
        qImage = False
        cvb = CvBridge()
        rospy.loginfo("Getting image from camera")
        try:
            rgb_image = cvb.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e


def get_depth(width, height, data):
    # This function retrieves a 3D from a 2D image coordinate
    if (height >= data.height) or (width >= data.width):
        return -1

    data_out = pc2.read_points(data, field_names=None, skip_nans=False, uvs=[[width, height]])
    int_data = next(data_out)
    return int_data


def readDepth(depth_data):
    # Callback for the point cloud topic
    global qDepth
    global pointcloud

    if qDepth:
        qDepth = False
        pointcloud = depth_data


def capture_callback(req):
    global qImage
    global qDepth

    status = False
    if req.capture:
        qImage = True
        qDepth = True
        status = True

    return captureResponse(status)


def block_detection(input_img):
    # ROS and OpenCV uses a BGR ordering (Blue, Green, Red) but most OpenCV functions work
    # with the RGB ordering (I know, weird). So you can swap colour channels as follow
    b, g, r = cv2.split(input_img)  # get b,g,r
    # rgb_img = cv2.merge([r, g, b])  # switch it to rgb
    cv2.imwrite('camera_image.jpg', input_img)

    # Display the RGB image (matplotlib is only used in jupyter, here you can use OpenCV builtin fns)
    #cv2.imshow("Image window", input_img)
    #cv2.waitKey(3)

    # Step 1. Segment red blocks in the image
    # As blocks are red, you only need to use the red channel for this
    red_channel = r.copy()
    # clip image by intensities, only retain pixels that within the 127-255 range
    # cv2.treshold will return a binary image where bright pixels denote retained red pixels
    _,img_bin = cv2.threshold(red_channel,200,255,cv2.THRESH_BINARY)

    # Display the RGB image
    if show_images:
        cv2.imshow("Tresholding the red channel", img_bin)
        cv2.waitKey(3)

    # Step 2. Apply Canny
    # convert the image to grayscale, blur it, and find edges
    # in the image using the Canny edge detector
    edges = cv2.Canny(img_bin,5,50)

    # Display extracted edges
    if show_images:
        cv2.imshow("Extracted edges", edges)
        cv2.waitKey(3)

    # Step 3. Find contours in the edge image, extract only those contours that are classified as external
    # See https://docs.opencv.org/3.1.0/d4/d73/tutorial_py_contours_begin.html for more info
    _, cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Step 4. Discard contours that have an area bigger than 10 squared pixels and less than 200 squared pixels.
    # If a contour within this range is found, append it and continue
    outCnt = []
    for c in cnts:
        area = cv2.contourArea(c)
        perimeter = cv2.arcLength(c,True)
        print "Area: " + str(area)
        print "Perimeter: " + str(perimeter)
        if perimeter >= 30 and perimeter <= 300:
            outCnt.append(c)
            # break  # Comment this if you want to detect more than one contour

    # Step 5. For the selected contour, retrieve the contour centroid location,
    # major axis length, minor axis length and major axis orientation
    # See http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT2/node3.html for more info
    # cv2.fitEllipse does the job in this case.
    # (x,y) is the contour centroid location
    # MA and ma are the major and minor axes
    # angle is the major axis orientation (angle is wrt the y-axis in the image) in degrees
    centroid = []
    angle = []
    out_img = input_img.copy()
    out_ellipse = []
    for c in outCnt:
        (x, y), (MA, ma), theta = cv2.fitEllipse(c)
        centroid.append((x, y))
        angle.append(theta)

        out_ellipse.append(cv2.fitEllipse(c))

        # Printing and displaying results
        cv2.drawContours(out_img, [c], -1, (0, 255, 0), 1)

        print "Centroid: " + str((x,y))
        print "Angle: " + str(angle)  # With respect to y in the image

    # Display contour on the rgb image
    # cv2.imshow("Contour", out_img)
    # cv2.waitKey(3)

    # Display ellipse on the out_img (this will show the contour and the fitted ellipse)
    for each_ellipse in out_ellipse:
        cv2.ellipse(out_img, each_ellipse, (255, 0, 0), 2)

    # cv2.imshow("Fitted Ellipse", out_img)
    # cv2.waitKey(3)
    cv2.imwrite('detection.jpg', out_img)

    return centroid, angle


def detector(input_img, pc, pubPose):
    global show_images

    centroids, angles = block_detection(input_img)
    print "Number of blocks detected: " + str(len(centroids))

    if len(centroids) > 0:
        # Always choose the first block in the list
        pt2D = centroids[0]
        theta = angles[0]
        # From degrees to radians
        # theta = radians(theta)

        # From the 2D location of the block, you can obtain the corresponding 3D
        # from the point cloud. The image and point cloud are logically indexed.
        # get_depth gets the job done!
        pt3D = get_depth(int(pt2D[0]), int(pt2D[1]), pc)

        print "Block centroid: " + str(pt2D)
        print "Block centroid 3D (wrt to Kinect): " + str(pt3D)
        print "Block angle: " + str(theta)  # With respect to y in the image

        # The 3D point is wrt to the Kinect, so you need to transform this point
        # to Baxter's base so you can use it with Baxter's Inverse Kinematics
        # service or MoveIt!
        pt3D_base = transformToBase(pt3D)
        print "Block centroid 3D (wrt to Baxter's base): " + str(pt3D_base)

        pubPose.publish(pt3D_base)
        rospy.loginfo("Pose published!!")

    else:
        rospy.loginfo("There are no blocks in front!")


# ********* ROS MAIN *********
def main():
    global camera_frame
    global rgb_image
    global pointcloud

    rospy.init_node('detect_blocks', anonymous=True)

    # Subscribers
    rospy.Subscriber("/camera/rgb/image_raw", Image, readImage, queue_size=1)
    rospy.Subscriber('/camera/depth/points', PointCloud2, readDepth, queue_size=1)

    # publishers
    pubPose = rospy.Publisher('/lab5_pkg/pose', PoseStamped, queue_size=1)

    # service
    rospy.Service('/lab5_pkg/capture', capture, capture_callback)
    rospy.loginfo("Node ready!")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if rgb_image is not None and pointcloud is not None:
            detector(rgb_image, pointcloud, pubPose)
            rgb_image = None
            pointcloud = None

        rate.sleep()

    cv2.destroyAllWindows()


# ********* MAIN *********
if __name__ == '__main__':
    sys.exit(main())
