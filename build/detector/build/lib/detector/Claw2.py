import serial
import time
from ultralytics import YOLO
import cv2 as cv

# YOLO model path
model = YOLO("/home/kemas/ros2_ws/src/detector/best.pt")

# Open video capture
cap = cv.VideoCapture(2)

# Camera parameters
camera_center_x = 325
tolerance = 20
camera_width = 640

# Define search area in cm
search_area_width_cm = 50
search_area_height_cm = 50

# Steps conversion (calibrated)
steps_per_cm_x = 50.0
steps_per_cm_y = 29.0

# Retry connection to Arduino
def connect_to_arduino():
    while True:
        try:
            arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)  # Wait for connection to stabilize
            print("Connected to Arduino.")
            return arduino
        except serial.SerialException as e:
            print(f"Failed to connect to Arduino: {e}. Retrying in 5 seconds...")
            time.sleep(5)

# Function to map pixel to cm
def pixel_to_cm(x_pixel, y_pixel, frame_width, frame_height):
    x_cm = (x_pixel / frame_width) * search_area_width_cm
    y_cm = (y_pixel / frame_height) * search_area_height_cm
    return x_cm, y_cm

# Connect to Arduino
arduino = connect_to_arduino()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Frame not captured.")
        break

    results = model(frame)
    frame_height, frame_width, _ = frame.shape
    print(f"Frame size: {frame_width}x{frame_height}")

    for result in results:
        print(f"Detected {len(result.boxes)} objects")

        for box in result.boxes:
            cls = int(box.cls)
            classes = result.names[cls]

            if classes == 'Red Payload Object':
                xyxy = box.xyxy[0]
                x1, y1, x2, y2 = map(int, xyxy.tolist())

                if 0 <= x1 <= frame_width and 0 <= x2 <= frame_width and 0 <= y1 <= frame_height and 0 <= y2 <= frame_height:
                    box_center_x = int((x1 + x2) / 2)
                    box_center_y = int((y1 + y2) / 2)
                    print(f"Center of Box - X: {box_center_x}, Y: {box_center_y}")

                    # Convert detected pixel coordinates to cm
                    target_x_cm, target_y_cm = pixel_to_cm(box_center_x, box_center_y, frame_width, frame_height)

                    # Send command to Arduino
                    try:
                        command = f"MOVE:{target_x_cm},{target_y_cm}\n"
                        arduino.write(command.encode())
                        print(f"Sent command to Arduino: {command}")

                        # Wait for claw to move
                        time.sleep(5)

                        # Move claw back to start
                        command = "MOVE:0,0\n"
                        arduino.write(command.encode())
                        print(f"Sent command to move back to start: {command}")
                    except serial.SerialException as e:
                        print(f"Serial write error: {e}")
                        arduino.close()
                        arduino = connect_to_arduino()

                    # Draw bounding box
                    cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv.putText(frame, f'{classes} ({target_x_cm:.2f}cm, {target_y_cm:.2f}cm)', 
                               (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    cv.imshow('YOLO Detection', frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv.destroyAllWindows()
arduino.close()
