import pytest
from robocup_real_manipulation.control_center_input import ApproachSession, observed_target, CLASSES


def status(phase='ready', active=False, stamp=100.):
    return dict(phase=phase, active=active, label='banana', classes=list(CLASSES),
        target=dict(label='banana', point_odom=[.3, .1, .6], point_map=[9., 8., .6],
            frame='map', capture_stamp=stamp, lidar_stamp=stamp-.02,
            lidar_oldest_stamp=stamp-.8, lidar_count=9,
            source_topic='/mid360/points_obstacles', point_method='yolo_mid360_bbox_cluster'))


def running():
    s = ApproachSession()
    s.update(status(), 100.)
    s.update(status('navigating', True), 100.1)
    return s


def test_success_uses_odom_original_capture_and_same_id_once():
    s = running()
    s.update(status('succeeded'), 100.2)
    s.update(status('succeeded'), 100.9)
    r = s.take(100.9, 100.2)
    assert r['target']['position'] == [.3, .1, .6]
    assert r['target']['frame'] == 'odom'
    assert r['target']['stamp'] == 100.
    assert r['target']['class_id'] == 5
    assert r['task_id'] == s.task
    assert s.take(100.95, 100.) is None


def test_succeeded_snapshot_at_startup_never_starts_job():
    s = ApproachSession()
    for now in [100., 100.5, 101.]:
        s.update(status('succeeded'), now)
        assert s.take(now, 90.) is None
    assert s.task is None


def test_startup_in_middle_of_approach_never_reconstructs_job():
    s = ApproachSession()
    s.update(status('navigating', True), 100.)
    s.update(status('succeeded'), 100.5)
    assert s.take(100.9, 90.) is None


def test_requires_measured_stop_after_completion_not_only_success_string():
    s = running()
    s.update(status('succeeded'), 100.2)
    assert s.take(100.9, None) is None
    assert s.take(100.7, 99.) is None
    assert s.take(100.9, 100.8) is None


@pytest.mark.parametrize('change,error', [
    (dict(point_odom=[float('nan'), 0., 0.]), 'INVALID_POINT'),
    (dict(capture_stamp=0.), 'STALE_TARGET'),
    (dict(capture_stamp=101.), 'STALE_TARGET'),
    (dict(lidar_count=2), 'UNVERIFIED_LIDAR'),
    (dict(point_method='manual'), 'UNVERIFIED_LIDAR'),
    (dict(lidar_oldest_stamp=90.), 'UNVERIFIED_LIDAR'),
])
def test_rejects_invalid_or_unmeasured_target(change, error):
    v = status(); v['target'].update(change)
    with pytest.raises(ValueError, match=error):
        observed_target(v, 100., 120.)


def test_class_schema_must_match_canonical_order():
    v = status(); v['classes'] = list(reversed(CLASSES))
    with pytest.raises(ValueError, match='CLASS_SCHEMA'):
        observed_target(v, 100., 120.)


def test_target_change_cancellation_and_stream_gap_prevent_handoff():
    for failure in ['target', 'cancel', 'gap']:
        s = running()
        if failure == 'target':
            v = status('succeeded'); v['target']['point_odom'][0] += .3
            s.update(v, 100.2)
        elif failure == 'cancel':
            s.update(status('canceled'), 100.2)
        else:
            s.update(status('succeeded'), 102.)
        assert s.take(102.5, 100.) is None


def test_new_observed_approach_has_new_uuid_and_frozen_start_measurement():
    s = running(); old = s.task
    s.update(status('ready'), 100.2)
    s.update(status('navigating', True, stamp=100.3), 100.4)
    assert s.task != old
    s.update(status('succeeded', stamp=100.5), 100.6)
    s.update(status('succeeded', stamp=100.7), 101.3)
    assert s.take(101.3, 100.6)['target']['stamp'] == 100.3


def test_clock_rollback_invalidates_task():
    s = running(); s.update(status('succeeded'), 99.9)
    assert s.take(100., 90.) is None
