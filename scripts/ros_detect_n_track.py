#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import Point

# Global state
model = YOLO('yolov8n.pt')  # Use absolute path if needed
tracker = None
tracking = False
detected_boxes = []
clicked_point = None
search = False

def click_event(event, x, y, flags, param):
    global clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)

def calc_closest_pixel(depth_img, x, y, w, h):
    height, width = depth_img.shape

    # Clamp the coordinates to stay within image bounds
    x1 = max(0, x)
    y1 = max(0, y)
    x2 = min(width, x+w)
    y2 = min(height, y+h)

    region = depth_img[y1:y2, x1:x2]
    region_no_zero = region[region > 0]  # filter noise in depth bounding box frame
    closest_pixel = np.min(region_no_zero)
    return closest_pixel

def main():
    global tracker, tracking, clicked_point, detected_boxes, search

    rospy.init_node('yolo_realsense_tracker', anonymous=True)

    # Publishers
    tracking_pub = rospy.Publisher('/tracked_status', Bool, queue_size=10)
    bb_center_pub = rospy.Publisher('/tracked_center', Point, queue_size=10)
    bb_depth_pub = rospy.Publisher('/tracked_depth', Int32, queue_size=10)
    search_pub = rospy.Publisher('/search_status', Bool, queue_size=10)

    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable color and depth streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Color stream
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)   # Depth stream

    # Start streaming
    pipeline.start(config)

    # Create alignment object to align depth to color
    align_to = rs.stream.color  # Align depth frame to color frame
    align = rs.align(align_to)

    cv2.namedWindow("YOLO Detection + Tracking")
    cv2.setMouseCallback("YOLO Detection + Tracking", click_event)

    rospy.loginfo("Press 'q' to quit or 'p' to stop tracking.")

    rate = rospy.Rate(30)  # 30 Hz
    while not rospy.is_shutdown():
        # Wait for a frame of data from the camera
        frames = pipeline.wait_for_frames()

        # Align the depth frame to the color frame
        aligned_frames = align.process(frames)

        # Get color and depth frames
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            rospy.logerr("Failed to grab frame.")
            break

        # Convert frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        color_image_cropped = color_image[0:480, 55:640]
        color_image_cropped_resized = cv2.resize(color_image_cropped, (color_image.shape[1], color_image.shape[0]))

        # frame_rgb = cv2.cvtColor(color_image_cropped_resized, cv2.COLOR_BGR2RGB)
        has_box = False
        bb_center_msg = Point()
        bb_depth_msg = Int32()

        if not tracking:
            # results = model(frame_rgb)
            results = model(color_image_cropped_resized)
            detected_boxes = []

            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()
                for box in boxes:
                    # if box.conf[0] > 0.5:  # nambahin ini
                    x1, y1, x2, y2 = map(int, box)
                    detected_boxes.append((x1, y1, x2, y2))
                    cv2.rectangle(color_image_cropped_resized, (x1, y1), (x2, y2), (255, 0, 0), 2)

            if clicked_point:
                cx, cy = clicked_point
                for x1, y1, x2, y2 in detected_boxes:
                    if x1 <= cx <= x2 and y1 <= cy <= y2:
                        bbox = (x1, y1, x2 - x1, y2 - y1)
                        tracker = cv2.TrackerCSRT_create()
                        tracker.init(color_image_cropped_resized, bbox)
                        tracking = True
                        break
                clicked_point = None

        else:
            success, bbox = tracker.update(color_image_cropped_resized)
            if success:
                x, y, w, h = map(int, bbox)
                cx, cy = x + w // 2, y + h // 2
                has_box = True
                bb_center_msg = Point(x=cx, y=cy, z=0)

                # # Get depth value (in milimeters)
                # depth_value = depth_image[cy,cx]  # by the center of the bounding box
                depth_value = calc_closest_pixel(depth_image, x, y, w, h)  # by closest pixel in the bounding box

                bb_depth_msg = Int32(data=depth_value)
                rospy.loginfo(f"Depth at center: {depth_value} mm")

                cv2.rectangle(color_image_cropped_resized, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(color_image_cropped_resized, "Tracking", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(color_image_cropped_resized, "Lost Tracking", (50, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                tracking = False

        # Publish status
        tracking_pub.publish(Bool(data=has_box))
        bb_center_pub.publish(bb_center_msg)
        bb_depth_pub.publish(bb_depth_msg)
        search_pub.publish(Bool(data=search))

        # Show frame
        cv2.imshow("Depth Image", depth_image)
        cv2.imshow("YOLO Detection + Tracking", color_image_cropped_resized)
        # cv2.imshow("cek", color_image)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            search = not search  # switch between True and False
        elif key == ord('p'):
            tracking = False
            tracker = None
            rospy.loginfo("Tracking canceled.")

        rate.sleep()

    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
