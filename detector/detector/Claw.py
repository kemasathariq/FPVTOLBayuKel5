import serial
import cv2
import numpy as np

# Initialize serial communication with Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600)  # Update the port if needed
cv2.namedWindow("Frame")

def send_position_to_arduino(offset_x, offset_y):
    """
    Sends the movement instructions to the Arduino.
    """
    command = f"{offset_x},{offset_y}\n"
    arduino.write(command.encode())
    print(f"Sent to Arduino: {command.strip()}")

def detect_red_object(frame):
    """
    Detect the red object in the frame and return its center coordinates.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the range for red color
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # Create masks for red
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Draw bounding box and center
        obj_center_x = x + w // 2
        obj_center_y = y + h // 2
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (obj_center_x, obj_center_y), 5, (255, 0, 0), -1)

        return obj_center_x, obj_center_y

    return None, None

def process_frame(frame):
    """
    Process the frame, detect the object, calculate offset, and send data to Arduino.
    """
    frame_height, frame_width, _ = frame.shape
    center_x = frame_width // 2
    center_y = frame_height // 2

    obj_center_x, obj_center_y = detect_red_object(frame)

    if obj_center_x is not None and obj_center_y is not None:
        # Calculate offsets in pixels
        offset_x = obj_center_x - center_x
        offset_y = obj_center_y - center_y

        # Convert offsets from pixels to steps (you need to calibrate this)
        steps_per_pixel_x = 0.5  # Example calibration: Adjust for your setup
        steps_per_pixel_y = 0.5  # Example calibration: Adjust for your setup

        move_x = int(offset_x * steps_per_pixel_x)
        move_y = int(offset_y * steps_per_pixel_y)

        # Send movement commands to Arduino
        send_position_to_arduino(move_x, move_y)
    else:
        print("No object detected")

    cv2.imshow("Frame", frame)

# Main loop
cap = cv2.VideoCapture(2)  # Update the camera index if needed

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break

    process_frame(frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
