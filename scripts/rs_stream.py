#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2

# Initialize the RealSense pipeline
pipeline = rs.pipeline()

# Configure the pipeline to use color and depth streams
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Color stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)   # Depth stream

# Start streaming
pipeline.start(config)

# Create alignment object to align depth to color
align_to = rs.stream.color  # Align depth frame to color frame
align = rs.align(align_to)

# Define the region to crop (x, y, width, height) -> buat ngecrop colored frame, soalnya setelah di align masih kurang pas
x, y, w, h = 55, 0, (640-55), 480

try:
    while True:
        # Wait for a frame of data from the camera
        frames = pipeline.wait_for_frames()

        # Align the depth frame to the color frame
        aligned_frames = align.process(frames)

        # Get the color and aligned depth frames
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image_cropped = color_image[y:y+h, x:x+w]   
        color_image_cropped_resized = cv2.resize(color_image_cropped, (color_image.shape[1], color_image.shape[0]))

        # Get the center of the frame
        height, width = depth_image.shape
        center_x, center_y = width // 2, height // 2

        # Get the depth value at the center of the frame (in millimeters)
        center_depth = depth_image[center_y, center_x]

        # Print depth value for the center pixel
        print(f"Depth at the center pixel: {center_depth} mm")

        # Convert the depth image to a color image for visualization (optional)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Display the color and depth images
        cv2.imshow('Color Stream', color_image_cropped_resized)
        cv2.imshow('Depth Stream', depth_colormap)

        # Check if the user presses 'q' to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
