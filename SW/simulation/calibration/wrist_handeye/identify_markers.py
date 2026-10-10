import cv2
import numpy as np
import pyrealsense2 as rs

CAMERA_SERIAL = "243222070076"

DICTIONARIES = {
    "AprilTag 16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "AprilTag 25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "AprilTag 36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "AprilTag 36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "ArUco 4x4": cv2.aruco.DICT_4X4_1000,
    "ArUco 5x5": cv2.aruco.DICT_5X5_1000,
    "ArUco 6x6": cv2.aruco.DICT_6X6_1000,
    "ArUco 7x7": cv2.aruco.DICT_7X7_1000,
}

detectors = {}

for name, dictionary_id in DICTIONARIES.items():
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    parameters = cv2.aruco.DetectorParameters()

    detectors[name] = cv2.aruco.ArucoDetector(
        dictionary,
        parameters
    )

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

    print("Camera started.")
    print("Point the camera at your calibration board.")
    print("Press Q to quit.")

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        results = []

        for name, detector in detectors.items():
            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None:
                results.append((name, len(ids), ids.flatten().tolist()))

        display = image.copy()

        y = 30

        for name, count, ids in results:
            text = f"{name}: {count} detected"

            cv2.putText(
                display,
                text,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            y += 30

        cv2.imshow("Marker Identification", display)

        if results:
            best = max(results, key=lambda result: result[1])

            name, count, ids = best

            cv2.putText(
                display,
                f"Best candidate: {name} ({count} markers)",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2
            )

            # Print only once when the user presses S.
            if cv2.waitKey(1) & 0xFF == ord("s"):
                print("\n=== MARKER IDENTIFICATION ===")
                print("Best candidate:", name)
                print("Detected markers:", count)
                print("Marker IDs:", ids)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    if started:
        pipeline.stop()

    cv2.destroyAllWindows()