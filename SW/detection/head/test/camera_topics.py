"""Select explicitly named Head RGB/aligned-depth pairs, never auto-select Wrist."""
PAIRS = (
    ('/head/camera/color/image_raw', '/head/camera/aligned_depth_to_color/image_raw'),
    ('/sensors/head/color/image_raw', '/sensors/head/aligned_depth_to_color/image_raw'),
    ('/head_camera/color/image_raw', '/head_camera/aligned_depth_to_color/image_raw'),
)


def select_topics(published, rgb=None, depth=None):
    if rgb or depth:
        if not rgb or not depth:
            raise ValueError('Explicit --rgb and --depth must be provided together')
        return rgb, depth
    for pair in PAIRS:
        if all(name in published for name in pair):
            return pair
    return PAIRS[0]  # A later-started default Head camera can still connect.


def input_status(rgb_publishers, depth_publishers, received):
    if not rgb_publishers:
        return 'NO_RGB_PUBLISHER (start Head camera or check ROS_DOMAIN_ID/topics)'
    if not received:
        return 'WAITING_RGB_FRAMES (publisher visible; no image callback received)'
    if not depth_publishers:
        return 'NO_DEPTH_PUBLISHER (enable aligned depth)'
    return ''
