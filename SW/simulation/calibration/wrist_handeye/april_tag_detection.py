import cv2
import numpy as np
import pyrealsense2 as rs

CAMERA_SERIAL = "243222070076"

# AprilTag 36h11 dictionary
dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_APRILTAG_36h11
)

parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

pipeline = rs.pipeline()
config = rs.config()

config.enable_device(CAMERA_SERIAL)

config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30
)

started = False

try:
    pipeline.start(config)
    started = True

    print("RealSense D435if started!")
    print("Show AprilTag 36h11 ID 0 to the camera.")
    print("Press Q to quit.")

    while True:
        frames = pipeline.wait_for_frames()
        frame = frames.get_color_frame()

        if not frame:
            continue

        image = np.asanyarray(frame.get_data())

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )

        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(
                image,
                corners,
                ids
            )

            detected_ids = ids.flatten().tolist()

            cv2.putText(
                image,
                f"Detected IDs: {detected_ids}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2
            )

        cv2.imshow("D435if AprilTag Detection", image)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            if ids is not None:
                print("Detected IDs:", ids.flatten().tolist())
            else:
                print("No AprilTag detected.")

        elif key == ord("q"):
            break

finally:
    if started:
        pipeline.stop()

    cv2.destroyAllWindows()