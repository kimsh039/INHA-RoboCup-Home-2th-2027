import pyrealsense2 as rs

CAMERA_SERIAL = "243222070076"

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
    profile = pipeline.start(config)
    started = True

    color_profile = profile.get_stream(
        rs.stream.color
    ).as_video_stream_profile()

    intr = color_profile.get_intrinsics()

    print("\n=== REALSENSE FACTORY INTRINSICS ===")
    print("Width:", intr.width)
    print("Height:", intr.height)

    print("fx:", intr.fx)
    print("fy:", intr.fy)
    print("cx:", intr.ppx)
    print("cy:", intr.ppy)

    print("Distortion model:", intr.model)
    print("Distortion coefficients:", list(intr.coeffs))

    print("\n=== TEAM CALIBRATION ===")
    print("fx: 619.88157")
    print("fy: 618.92987")
    print("cx: 338.06437")
    print("cy: 245.83783")

finally:
    if started:
        pipeline.stop()