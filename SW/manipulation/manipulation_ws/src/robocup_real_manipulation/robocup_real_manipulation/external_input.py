"""Convert existing Head classification + fusion 3D; never estimate/re-detect."""
import math


def capture_key(header):
    return header.stamp.sec, header.stamp.nanosec


def check_pair(head, point, config, now, max_age):
    if capture_key(head.header) != capture_key(point.header):
        raise ValueError('CAPTURE_MISMATCH')
    stamp = head.header.stamp.sec + head.header.stamp.nanosec / 1e9
    if stamp <= 0 or not -config['future_tolerance_s'] <= now-stamp <= max_age:
        raise ValueError('STALE_OR_FUTURE_INPUT')
    if not head.valid or not head.measured or head.reason or not head.target_id:
        raise ValueError('UNVERIFIED_HEAD_TARGET')
    names = config['model_names']
    if not 0 <= head.class_id < len(names) or names[head.class_id] != head.class_name:
        raise ValueError('CLASS_SCHEMA_MISMATCH')
    rule = config['classes'].get(head.class_name)
    if not rule or not rule['enabled'] or rule['class_id'] != head.class_id:
        raise ValueError('UNSUPPORTED_CLASS')
    if not math.isfinite(head.confidence) or head.confidence < config['min_confidence']:
        raise ValueError('LOW_CONFIDENCE')
    p = point.point
    if not point.header.frame_id or not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
        raise ValueError('INVALID_3D_POINT')
    return head.target_id


def auto_candidate(targets, approaches, attempted):
    for task_id in targets:
        result = approaches.get(task_id)
        if task_id not in attempted and result and result.outcome == 'SUCCESS' and result.base_stopped:
            return task_id
    return None
