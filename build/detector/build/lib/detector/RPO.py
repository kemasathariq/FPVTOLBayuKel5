from ultralytics import YOLO
import cv2 as cv

model = YOLO("/home/kemas/ros2_ws/src/detector/best.pt")

cap = cv.VideoCapture(0)

center_x = 325 
tolerance = 20 

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
                    print(f"Valid bounding box: ({x1}, {y1}), ({x2}, {y2})")
                    
                    box_center_x = int((x1 + x2) / 2)
                    box_center_y = int((y1 + y2) / 2)

                    print(f"Center of Box - X: {box_center_x}, Y: {box_center_y}")

                    if abs(box_center_x - center_x) <= tolerance:
                        cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv.putText(frame, classes, (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        
                        print(f"Red Payload Object detected near center X at: {box_center_x}")
                    else:
                        print(f"Object detected but not near center X: {box_center_x}")
                else:
                    print(f"Bounding box out of bounds: ({x1}, {y1}), ({x2}, {y2})")

    cv.imshow('YOLO Detection', frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
