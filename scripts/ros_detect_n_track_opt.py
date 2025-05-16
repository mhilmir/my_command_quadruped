#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from ultralytics import YOLO
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch

model = YOLO('yolov8n.pt')  # Use absolute path if needed
tracker = None
tracking = False
detected_boxes = []
clicked_point = None
search = False
color_image = None
depth_image = None
bridge = CvBridge()
frame_count = 0  # For YOLO detection interval

def click_event(event, x, y, flags, param):
    global clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)

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

def main():
    global tracker, tracking, clicked_point, detected_boxes, search, color_image, depth_image, frame_count

    rospy.init_node('deteck_n_track_opt_node', anonymous=True)

    tracking_pub = rospy.Publisher('/tracked_status', Bool, queue_size=10)
    bb_center_pub = rospy.Publisher('/tracked_center', Point, queue_size=10)
    bb_depth_pub = rospy.Publisher('/tracked_depth', Int32, queue_size=10)
    search_pub = rospy.Publisher('/search_status', Bool, queue_size=10)

    rospy.Subscriber('/camera/color/image_raw', Image, color_callback)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback)

    if torch.cuda.is_available():
        rospy.loginfo("CUDA is available, using GPU")
        model.to('cuda')
    else:
        rospy.logwarn("CUDA not available, running on CPU (slow!)")

    cv2.namedWindow("YOLO Detection + Tracking")
    cv2.setMouseCallback("YOLO Detection + Tracking", click_event)

    rospy.loginfo("Press 'q' to quit or 'p' to stop tracking.")

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

        if not tracking:
            if frame_count % 3 == 0:
                results = model(img_for_model, conf=0.4, verbose=False)
                detected_boxes = []
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
                cx, cy = clicked_point
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
                rospy.loginfo(f"Depth at center: {depth_value} mm")
            else:
                cv2.putText(color_image, "Lost Tracking", (50, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                tracking = False

        # Publish
        tracking_pub.publish(Bool(data=has_box))
        bb_center_pub.publish(bb_center_msg)
        bb_depth_pub.publish(bb_depth_msg)
        search_pub.publish(Bool(data=search))

        # Display
        cv2.imshow("YOLO Detection + Tracking", color_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            search = not search
        elif key == ord('p'):
            tracking = False
            tracker = None
            rospy.loginfo("Tracking canceled.")

        frame_count += 1
        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
