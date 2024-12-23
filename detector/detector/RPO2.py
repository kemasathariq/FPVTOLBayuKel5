from ultralytics import YOLO
import cv2 as cv
import serial
import time

arduino = serial.Serial('/dev/ttyUSB0', 9600)

model = YOLO("/home/kemas/ros2_ws/src/detector/best.pt")

cap = cv.VideoCapture(0)

center_x = 325 
tolerance = 30

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Frame not captured.")
        break

    results = model(frame)

    frame_height, frame_width, _ = frame.shape
    print(f"Frame size: {frame_width}x{frame_height}")

    object_detected = False
    for result in results:
        print(f"Detected {len(result.boxes)} objects")

        for box in result.boxes:
            cls = int(box.cls)
            classes = result.names[cls]

            if classes == 'Red Payload Object':
                xyxy = box.xyxy[0]  
                x1, y1, x2, y2 = map(int, xyxy.tolist()) 

                box_center_x = int((x1 + x2) / 2)

                if abs(box_center_x - center_x) <= tolerance:
                    object_detected = True
                    print(f"Red Payload Object detected near center X at: {box_center_x}")
                    arduino.write(b'stop\n')
                else:
                    print(f"Object detected but not near center X: {box_center_x}")

    if not object_detected:
        arduino.write(b'start\n')

    cv.imshow('YOLO Detection', frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
