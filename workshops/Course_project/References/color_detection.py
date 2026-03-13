#!/usr/bin/env python3
"""
Color Detection Program using OpenCV

This program detects the largest colored object (red, green, or blue) in the camera frame
and prints the detected color. It also shows the live camera feed with the detected color
displayed on top.

Requirements:
- OpenCV (cv2)
- NumPy

Installation:
pip install opencv-python numpy

Press ESC to exit the program.
"""

import cv2
import numpy as np

# Initialize camera (0 = default webcam)
cap = cv2.VideoCapture(0)

def detect_color(frame):
    """
    Detects the largest colored object in the frame.

    Args:
        frame (numpy.ndarray): BGR image from the camera.

    Returns:
        str: Detected color: 'RED', 'GREEN', 'BLUE', or 'NONE' if no significant color detected.
    """
    # Convert BGR image to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define HSV ranges for colors
    colors = {
        "RED": ([0, 120, 70], [10, 255, 255]),    # Lower and upper HSV range for RED
        "GREEN": ([40, 50, 50], [80, 255, 255]),  # Lower and upper HSV range for GREEN
        "BLUE": ([100, 150, 0], [140, 255, 255])  # Lower and upper HSV range for BLUE
    }

    biggest_area = 0          # Keep track of the largest object found
    detected_color = "NONE"   # Default if no color is detected

    # Loop through each color
    for color_name, (lower, upper) in colors.items():
        lower = np.array(lower)
        upper = np.array(upper)

        # Create a mask for the color
        mask = cv2.inRange(hsv, lower, upper)

        # Reduce noise using Gaussian blur
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through detected contours
        for cnt in contours:
            area = cv2.contourArea(cnt)  # Calculate contour area

            # Consider only large objects (ignore noise) and select the largest one
            if area > 800 and area > biggest_area:
                biggest_area = area
                detected_color = color_name

    return detected_color

# Main loop
while True:
    # Capture a frame from the camera
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame from camera. Exiting.")
        break

    # Detect color in the frame
    color = detect_color(frame)

    # Print detected color
    print("Detected Color:", color)

    # Display detected color on the video frame
    cv2.putText(frame, color, (50, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                (0, 255, 0), 2)

    # Show the camera feed
    cv2.imshow("Color Detection", frame)

    # Exit on ESC key
    if cv2.waitKey(1) & 0xFF == 27:  # 27 = ESC
        break

# Release camera and close windows
cap.release()
cv2.destroyAllWindows()
