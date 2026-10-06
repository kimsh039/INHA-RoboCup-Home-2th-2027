import cv2
import numpy as np
import pyrealsense2 as rs


pipeline = rs.pipeline()
config = rs.config()

# RGB
config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30,
)

# Depth
config.enable_stream(
    rs.stream.depth,
    640,
    480,
    rs.format.z16,
    30,
)

profile = pipeline.start(config)

# Align depth image to RGB image
align = rs.align(rs.stream.color)
# RealSense depth filters
spatial_filter = rs.spatial_filter()
temporal_filter = rs.temporal_filter()
hole_filling_filter = rs.hole_filling_filter()
print("RealSense RGB + Depth started.")
print("Press Q to quit.")

try:
    while True:

        frames = pipeline.wait_for_frames()

        # Align depth to the color camera
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        # Apply filters
        filtered_depth = spatial_filter.process(depth_frame)
        filtered_depth = temporal_filter.process(filtered_depth)
        filtered_depth = hole_filling_filter.process(filtered_depth)

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(
            color_frame.get_data()
        )

        depth_image = np.asanyarray(
            filtered_depth.get_data()
        )

        # Convert depth to a visible color image
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(
                depth_image,
                alpha=0.03
            ),
            cv2.COLORMAP_JET
        )

        # Get depth at center of image
        center_x = 320
        center_y = 240

        # Use a small square around the center instead of one pixel
        depth_frame_filtered = filtered_depth.as_depth_frame()

        depth_values = []

        window_size = 15

        for y in range(center_y - window_size, center_y + window_size + 1):
            for x in range(center_x - window_size, center_x + window_size + 1):

                d = depth_frame_filtered.get_distance(x, y)

                # Keep only reasonable depth values
                if 0.15 < d < 4.0:
                    depth_values.append(d)

        if len(depth_values) > 0:
            distance = float(np.median(depth_values))
        else:
            distance = 0.0
        # Draw center point
        cv2.circle(
            color_image,
            (center_x, center_y),
            5,
            (0, 0, 255),
            -1
        )

        cv2.putText(
            color_image,
            f"Center depth: {distance:.2f} m",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        cv2.rectangle(
            color_image,
            (
                center_x - window_size,
                center_y - window_size
            ),
            (
                center_x + window_size,
                center_y + window_size
            ),
            (0, 255, 0),
            2
        )

        cv2.imshow(
            "RGB",
            color_image
        )

        cv2.imshow(
            "Depth",
            depth_colormap
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()