#!/usr/bin/env python3

import numpy as np
import cv2
from ultralytics import YOLO
import torch
### Always import torch and ultralytics before any ROS-related imports
import rospy
from std_msgs.msg import Bool, Int32, Empty
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

model = YOLO('yolov8n.pt')  # Use absolute path if needed
tracker = None
tracking = False
detected_boxes = []
clicked_point = None
color_image = None
depth_image = None
bridge = CvBridge()
frame_count = 0  # For YOLO detection interval
yolo_enabled = False  # New flag for toggling YOLO detection

def calc_closest_pixel(depth_img, x, y, w, h):
    height, width = depth_img.shape
    x1 = max(0, x)
    y1 = max(0, y)
    x2 = min(width, x+w)
    y2 = min(height, y+h)
    region = depth_img[y1:y2, x1:x2]
    region_no_zero = region[region > 0]
    return int(np.min(region_no_zero)) if region_no_zero.size > 0 else 0

def color_callback(msg):
    global color_image
    try:
        color_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        rospy.logerr(f"Failed to convert color image: {e}")

def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, msg.encoding)
    except Exception as e:
        rospy.logerr(f"Failed to convert depth image: {e}")

def mouse_callback(msg):
    global clicked_point
    clicked_point = msg

def yolo_enabled_callback(msg):
    global yolo_enabled
    yolo_enabled = msg.data
    rospy.loginfo(f"YOLO Detection {'Enabled' if yolo_enabled else 'Disabled'}")

def cancel_tracking_callback(msg):
    global tracker, tracking
    tracking = False
    tracker = None
    rospy.loginfo("Tracking canceled.")

def main():
    global tracker, tracking, clicked_point, detected_boxes, color_image, depth_image, frame_count, yolo_enabled

    rospy.init_node('quadruped_cam_node', anonymous=True)

    tracking_pub = rospy.Publisher('/tracked_status', Bool, queue_size=10)
    bb_center_pub = rospy.Publisher('/tracked_center', Point, queue_size=10)
    bb_depth_pub = rospy.Publisher('/tracked_depth', Int32, queue_size=10)
    image_pub = rospy.Publisher('/camera/quadruped/front_cam', Image, queue_size=1)

    rospy.Subscriber('/camera/color/image_raw', Image, color_callback)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback)
    rospy.Subscriber("/camera/quadruped/mouse_click", Point, mouse_callback)
    rospy.Subscriber('/yolo_enabled', Bool, yolo_enabled_callback)
    rospy.Subscriber('/cancel_tracking', Empty, cancel_tracking_callback)

    if torch.cuda.is_available():
        rospy.loginfo("CUDA is available, using GPU")
        model.to('cuda')
    else:
        rospy.logwarn("CUDA not available, running on CPU (slow!)")

    # rospy.loginfo("Press 'q' to quit or 'p' to stop tracking.")

    rate = rospy.Rate(30)  # 30 Hz
    while not rospy.is_shutdown():
        if color_image is None or depth_image is None:
            rate.sleep()
            continue

        # Downscale for speed
        display_image = cv2.resize(color_image.copy(), (416, 416))
        img_for_model = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
        has_box = False
        bb_center_msg = Point()
        bb_depth_msg = Int32()
        detected_boxes = []

        if not tracking:
            # if yolo_enabled and frame_count % 3 == 0:
            if yolo_enabled:
                results = model(img_for_model, conf=0.4, verbose=False)
                # detected_boxes = []
                h_scale = color_image.shape[0] / 416
                w_scale = color_image.shape[1] / 416

                for result in results:
                    boxes = result.boxes.xyxy.cpu().numpy()
                    for box in boxes:
                        x1, y1, x2, y2 = map(int, box)
                        x1 = int(x1 * w_scale)
                        y1 = int(y1 * h_scale)
                        x2 = int(x2 * w_scale)
                        y2 = int(y2 * h_scale)
                        detected_boxes.append((x1, y1, x2, y2))
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (255, 0, 0), 2)

            if clicked_point:
                cx = clicked_point.x
                cy = clicked_point.y
                # print(cx, cy)
                for x1, y1, x2, y2 in detected_boxes:
                    if x1 <= cx <= x2 and y1 <= cy <= y2:
                        bbox = (x1, y1, x2 - x1, y2 - y1)
                        tracker = cv2.TrackerCSRT_create()
                        tracker.init(color_image, bbox)
                        tracking = True
                        break
                clicked_point = None
        else:
            success, bbox = tracker.update(color_image)
            if success:
                x, y, w, h = map(int, bbox)
                cx, cy = x + w // 2, y + h // 2
                has_box = True
                bb_center_msg = Point(x=cx, y=cy, z=0)
                depth_value = calc_closest_pixel(depth_image, x, y, w, h)
                bb_depth_msg = Int32(data=depth_value)
                cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(color_image, "Tracking", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                rospy.loginfo(f"Closest Depth data in bb: {depth_value} mm")
            else:
                cv2.putText(color_image, "Lost Tracking", (50, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                tracking = False

        # Publish
        tracking_pub.publish(Bool(data=has_box))
        bb_center_pub.publish(bb_center_msg)
        bb_depth_pub.publish(bb_depth_msg)
        
        image_msg = bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        image_pub.publish(image_msg)

        # Display
        # cv2.imshow("YOLO Detection + Tracking", color_image)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
