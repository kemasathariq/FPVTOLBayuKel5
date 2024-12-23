from ultralytics import YOLO
import cv2 as cv
import serial
import time

# Connect to Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Allow time for Arduino to reset

# Load YOLO model
model = YOLO("/home/kemas/ros2_ws/src/detector/best.pt")

# Open webcam
cap = cv.VideoCapture(2)

center_x = 325  # Center of the frame (x-axis)
center_y = 240  # Center of the frame (y-axis)
tolerance = 20  # Tolerance for how close the object should be to the center

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Frame not captured.")
        break

    # Run YOLO object detection
    results = model(frame)

    object_detected_near_center = False

    for result in results:
        for box in result.boxes:
            cls = int(box.cls)
            class_name = result.names[cls]

            if class_name == 'Red Payload Object':
                xyxy = box.xyxy[0]
                x1, y1, x2, y2 = map(int, xyxy.tolist())

                box_center_x = int((x1 + x2) / 2)
                box_center_y = int((y1 + y2) / 2)

                if abs(box_center_x - center_x) <= tolerance and abs(box_center_y - center_y) <= tolerance:
                    object_detected_near_center = True
                    print(f"Red Payload Object detected near center at X: {box_center_x}, Y: {box_center_y}")

                    # Send stop command to Arduino
                    arduino.write(b'stop\n')

                    # Draw bounding box and label
                    cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv.putText(frame, class_name, (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

    if not object_detected_near_center:
        arduino.write(b'start\n')

    cv.imshow('YOLO Detection', frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
