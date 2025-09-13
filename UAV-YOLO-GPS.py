import cv2
from ultralytics import YOLO
import time
import math
import threading
import queue

# Load YOLOv8 OpenVINO-optimized model
ov_model=YOLO('C:\YOLOV8\V2_openvino_model\\')

# Function to calculate approximate real-world distance
def RealDistance(totalPixel, height, pitch):
    #Convert pixels to m (assuming scale factor)
    realPiksel = totalPixel * 0.026458333333333
    #Basic distance calculation (rough approximation)
    distance = realPiksel + (height * (math.tan(90 - pitch)))
    return distance


# Function to estimate GPS coordinates of a detected object
def calcDist(height, pitch, lat, lon, yaw, targetCenterY, frameY):
    pixel = 100000 * 0.026458333333333
    a = height * math.tan(90 - pitch) # Longtitue


    b = math.tan(90 - yaw) * a # Latitute

    # Latitude difference (approximation)
    latDiff = b * 8.983 * math.pow(10, -6)
    # Longitude difference (approximation)
    lonDiff = a * 8.983 * math.pow(10, -6)

    # Offset from frame center
    y = (frameY // 2) - targetCenterY
    x = math.tan(90 - pitch) / math.tan(90 - yaw)

    # Pixel-based offsets
    latDiffP = x * 8.983 * math.pow(10, -6)
    lonDiffP = y * 8.983 * math.pow(10, -6)

    # Adjust latitude based on yaw
    if yaw == -90 or yaw == 90:
        detectedLat = lat
    elif -90 < yaw < 90:
        detectedLat = lat + latDiff
    else:
        detectedLat = lat - latDiff

    # Adjust longitude based on yaw
    if yaw == 0 or yaw == 180:
        detectedLon = lon
    elif 0 < yaw < 180:
        detectedLon = lon - lonDiff
    else:
        detectedLon = lon + lonDiff

    # Apply pixel-based adjustments
    if latDiffP < 0:
        detectedLat -= latDiffP
    elif latDiffP > 0:
        detectedLat += latDiffP

    if lonDiffP < 0:
        detectedLon -= lonDiffP
    elif lonDiffP > 0:
        detectedLon += lonDiffP

    return detectedLat, detectedLon

# Thread: Reads frames from camera and adds to queue
def read_frame(cam, frame_queue, stop_event):
    while not stop_event.is_set():
        ret, frame = cam.read()
        if not ret:
            break
        if frame_queue.full():
            frame_queue.get()  # Drop oldest frame if queue is full
        frame_queue.put(frame)
        time.sleep(0.000001)  # Tiny sleep to reduce CPU load


# Thread: Processes frames using YOLO model
def process_frame(cam, frame_queue, stop_event):
    while not stop_event.is_set():
        if not frame_queue.empty():
            frame = frame_queue.get()
            frame = cv2.resize(frame, (1080, 720))

            results = ov_model(frame)
            detections = results[0].boxes
            class_names = ov_model.names
            height, width, _ = frame.shape
            center_x, center_y = width // 2, height // 2

            i = 50
            # Process detected objects
            for box in detections:
                class_id = int(box.cls)
                class_name = class_names[class_id]

                # Bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0]

                # Object center point
                targetCenterX = int((x2 - x1) / 2) + x1
                targetCenterY = int((y2 - y1) / 2) + y1

                # Approximate object size in pixels
                distancePixel = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

                # Color depends on detected class
                if class_name == 'patika':
                    color = (0, 250, 0)  # Green
                elif class_name == 'asfalt':
                    color = (255, 0, 0)  # Blue
                else:
                    color = (0, 0, 255)  # Default Red

                # Only process confident detections
                if confidence >= 0.4:
                    # Draw detection visuals
                    cv2.circle(frame, (targetCenterX, targetCenterY), 5, (0, 0, 255), -1)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, f'{class_name} {confidence:.2f}', (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

                    # Write pixel distance and real distance
                    cv2.putText(frame, f"Distance:{distancePixel:.2f} p", (50, i),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), thickness=2)
                    cv2.putText(frame, f"Real Distance:{RealDistance(distancePixel, 30, 30):.2f} m",(500, i), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

                    # Center marker
                    cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

                    i += 25

            # Show processed frame
            cv2.imshow("Processed Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break

# Main function
def main():

    cam = cv2.VideoCapture(0)

    frame_queue = queue.Queue(maxsize=3)
    stop_event = threading.Event()

    # Start threads for reading and processing frames
    read_thread = threading.Thread(target=read_frame, args=(cam, frame_queue, stop_event))
    process_thread = threading.Thread(target=process_frame, args=(cam, frame_queue, stop_event))

    read_thread.start()
    process_thread.start()

    try:
        read_thread.join()
        process_thread.join()
    except KeyboardInterrupt:
        stop_event.set()
        read_thread.join()
        process_thread.join()

    # Release resources
    cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
