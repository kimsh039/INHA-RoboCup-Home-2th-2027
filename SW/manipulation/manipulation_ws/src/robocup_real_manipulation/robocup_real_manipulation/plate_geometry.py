"""SAM provides masks, not plate classes. Accept only measured empty plate geometry."""
import cv2
import numpy as np
from .core import require


def plate_disk(depth, mask, info, rotation, translation, support_z, settings):
    binary = np.asarray(mask, bool)
    require(binary.shape == depth.shape and binary.any(), 'PLATE_MASK_EMPTY')
    require(not (binary[0].any() or binary[-1].any() or binary[:, 0].any() or binary[:, -1].any()), 'PLATE_CLIPPED')
    contours, _ = cv2.findContours(binary.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    require(len(contours) == 1, 'PLATE_MULTIPLE_COMPONENTS')
    contour = contours[0]
    area, perimeter = cv2.contourArea(contour), cv2.arcLength(contour, True)
    require(perimeter > 0 and 4*np.pi*area/perimeter**2 >= settings.get('min_circularity', .65), 'PLATE_SHAPE')
    require(area/max(1., cv2.contourArea(cv2.convexHull(contour))) >= .9, 'PLATE_CONCAVE_MASK')
    k = np.asarray(info.k).reshape(3, 3)
    require(k[0, 0] > 0 and k[1, 1] > 0 and not any(info.d), 'PLATE_RECTIFIED_INTRINSICS')
    y, x = np.nonzero(binary)
    z = depth[y, x]
    valid = np.isfinite(z) & (z > .15) & (z < 1.5)
    require(valid.mean() >= .9, 'PLATE_DEPTH_COVERAGE')
    x, y, z = x[valid], y[valid], z[valid]
    camera = np.column_stack(((x-k[0, 2])*z/k[0, 0], (y-k[1, 2])*z/k[1, 1], z))
    world = camera @ rotation.T+translation
    require(float(rotation[2, 2]) < -np.cos(np.deg2rad(15)), 'PLATE_VIEW_NOT_TOPDOWN')
    diameter = np.ptp(world[:, :2], axis=0)
    require(np.all(diameter >= settings.get('min_diameter_m', .12)) and
            np.all(diameter <= settings.get('max_diameter_m', .40)), 'PLATE_SIZE')
    # Raised rim is allowed, a bare table or tall object is not a plate.
    height = world[:, 2]-support_z
    floor = float(np.percentile(height, 30))
    require(settings.get('min_height_m', .002) <= floor <= settings.get('max_height_m', .04), 'PLATE_SUPPORT_HEIGHT')
    require(float(np.percentile(height, 95)) <= .065, 'PLATE_TALL_CONTENT')
    flat = np.abs(height-floor) <= settings.get('plane_tolerance_m', .006)
    require(float(flat.mean()) >= .55, 'PLATE_EMPTY_FLOOR_MISSING')
    free = np.zeros(depth.shape, np.uint8)
    free[y[flat], x[flat]] = 1
    # Unknown depth, raised rim and contents remain excluded; no hole filling.
    distance = cv2.distanceTransform(free, cv2.DIST_L2, 5)
    _, radius_px, _, (cx, cy) = cv2.minMaxLoc(distance)
    scale = float(np.min(z))/max(k[0, 0], k[1, 1]) * abs(float(rotation[2, 2]))
    radius = max(0., radius_px*scale-settings.get('edge_margin_m', .015))
    require(radius >= settings.get('min_free_radius_m', .025), 'PLATE_NO_FREE_DISK')
    cz = float(depth[cy, cx])
    point = np.array([(cx-k[0, 2])*cz/k[0, 0], (cy-k[1, 2])*cz/k[1, 1], cz])
    return point, radius, float(np.std(height[flat]))
