import cv2
import numpy as np
import pyrealsense2 as rs

CAMERA_SERIAL = "243222070076"

# AprilTag 36h11 detector
dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_APRILTAG_36h11
)

detector = cv2.aruco.ArucoDetector(
    dictionary,
    cv2.aruco.DetectorParameters()
)

# RealSense configuration
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

    print("Board inspection started.")
    print("Press S to print detected marker positions.")
    print("Press Q to quit.")

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        image = np.asanyarray(
            color_frame.get_data()
        ).copy()

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )

        corners, ids, _ = detector.detectMarkers(gray)

        marker_positions = []

        if ids is not None:

            cv2.aruco.drawDetectedMarkers(
                image,
                corners,
                ids
            )

            for marker_corners, marker_id in zip(
                corners,
                ids.flatten()
            ):

                points = marker_corners.reshape(4, 2)

                # Display OpenCV's corner order
                for corner_index, (px, py) in enumerate(points):
                    cv2.circle(
                        image,
                        (int(px), int(py)),
                        4,
                        (0, 0, 255),
                        -1
                    )

                    cv2.putText(
                        image,
                        str(corner_index),
                        (int(px) + 5, int(py) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (255, 0, 0),
                        1
                    )

                center = np.mean(points, axis=0)

                x = int(center[0])
                y = int(center[1])

                marker_positions.append(
                    (int(marker_id), x, y)
                )

                cv2.putText(
                    image,
                    f"ID {marker_id}",
                    (x - 20, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2
                )

        count = len(marker_positions)

        cv2.putText(
            image,
            f"Detected: {count}/36",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.imshow("AprilTag Board Inspection", image)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):

            print("\n=== BOARD GEOMETRY CHECK ===")

            positions = {
                marker_id: (x, y)
                for marker_id, x, y in marker_positions
            }

            for marker_id in [0, 1, 6, 7]:
                if marker_id in positions:
                    x, y = positions[marker_id]

                    print(
                        f"ID {marker_id}: "
                        f"x={x}, y={y}"
                    )
                else:
                    print(f"ID {marker_id}: not detected")

        elif key == ord("q"):
            break

finally:
    if started:
        pipeline.stop()

    cv2.destroyAllWindows()