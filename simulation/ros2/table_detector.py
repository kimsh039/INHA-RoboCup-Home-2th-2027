#!/usr/bin/env python3

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray

from scipy.spatial import cKDTree, ConvexHull


# ============================================================
# Physical table model
# ============================================================

TABLE_LENGTH = 1.60
TABLE_WIDTH = 0.80
TABLE_HEIGHT = 0.57

MAX_DETECTION_RANGE = 5.0


class TableDetector(Node):

    def __init__(self):

        super().__init__('table_detector')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/mid360/points_filtered',
            self.pointcloud_callback,
            qos_profile_sensor_data
        )

        self.tables_pub = self.create_publisher(
            PointCloud2,
            '/table_detection/tables',
            qos_profile_sensor_data
        )

        self.centers_pub = self.create_publisher(
            PoseArray,
            '/table_detection/centers',
            10
        )

        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/table_detection/table_markers',
            10
        )

        self.get_logger().info(
            'Automatic edge-based table detector started.'
        )

    # ============================================================
    # XY clustering
    # ============================================================
    def remove_wall_lines(self, points):

        if len(points) < 2:
            return points

        remaining = points.copy()

        rng = np.random.default_rng(42)

        MAX_WALLS = 8
        RANSAC_TRIALS = 350

        LINE_DISTANCE_THRESHOLD = 0.045
        MIN_WALL_POINTS = 35

        # Intentionally larger than the 1.6 m table length.
        MIN_WALL_LENGTH = 2.0

        for _ in range(MAX_WALLS):

            if len(remaining) < MIN_WALL_POINTS:
                break

            xy = remaining[:, :2]

            best_indices = None
            best_count = 0
            best_span = 0.0

            for _ in range(RANSAC_TRIALS):

                ids = rng.choice(
                    len(xy),
                    size=2,
                    replace=False
                )

                p1 = xy[ids[0]]
                p2 = xy[ids[1]]

                direction = p2 - p1

                norm = np.linalg.norm(direction)

                if norm < 0.20:
                    continue

                direction = direction / norm

                normal = np.array([
                    -direction[1],
                    direction[0]
                ])

                distances = np.abs(
                    (xy - p1) @ normal
                )

                indices = np.where(
                    distances
                    <= LINE_DISTANCE_THRESHOLD
                )[0]

                if len(indices) < MIN_WALL_POINTS:
                    continue

                projections = (
                    xy[indices] - p1
                ) @ direction

                span = float(
                    projections.max()
                    - projections.min()
                )

                if span < MIN_WALL_LENGTH:
                    continue

                if len(indices) > best_count:

                    best_indices = indices
                    best_count = len(indices)
                    best_span = span

            if best_indices is None:
                break

            self.get_logger().info(
                f'Removing wall line: '
                f'{best_count} points | '
                f'span {best_span:.2f} m'
            )

            keep = np.ones(
                len(remaining),
                dtype=bool
            )

            keep[best_indices] = False

            remaining = remaining[keep]

        return remaining
    
    def cluster_xy(
        self,
        points,
        radius=0.25,
        min_points=25
    ):

        if len(points) == 0:
            return []

        xy = points[:, :2]

        tree = cKDTree(xy)

        visited = np.zeros(
            len(points),
            dtype=bool
        )

        clusters = []

        for start in range(len(points)):

            if visited[start]:
                continue

            queue = [start]
            visited[start] = True

            indices = []

            while queue:

                current = queue.pop()

                indices.append(current)

                neighbors = tree.query_ball_point(
                    xy[current],
                    radius
                )

                for neighbor in neighbors:

                    if not visited[neighbor]:

                        visited[neighbor] = True
                        queue.append(neighbor)

            if len(indices) >= min_points:

                clusters.append(
                    points[indices]
                )

        return clusters

    # ============================================================
    # Surface statistics
    # ============================================================

    def surface_statistics(
        self,
        cluster
    ):

        z = cluster[:, 2]

        z_mean = float(
            np.mean(z)
        )

        z_std = float(
            np.std(z)
        )

        robust_z_span = float(
            np.percentile(z, 95)
            -
            np.percentile(z, 5)
        )

        hull_area = 0.0

        try:

            hull = ConvexHull(
                cluster[:, :2]
            )

            hull_area = float(
                hull.volume
            )

        except Exception:

            pass

        return (
            z_mean,
            z_std,
            robust_z_span,
            hull_area
        )

    # ============================================================
    # Angle helpers
    # ============================================================

    def normalize_yaw(
        self,
        yaw
    ):

        while yaw >= math.pi / 2.0:
            yaw -= math.pi

        while yaw < -math.pi / 2.0:
            yaw += math.pi

        return yaw

    def angle_difference_90(
        self,
        angle_a,
        angle_b
    ):

        """
        Difference between two rectangular orientations.

        A rectangle has equivalent edge directions every 90 deg,
        so 0 deg and 90 deg describe the same pair of edge families.
        """

        difference = abs(
            angle_a - angle_b
        )

        while difference >= math.pi / 2.0:
            difference -= math.pi / 2.0

        return min(
            difference,
            math.pi / 2.0 - difference
        )

    # ============================================================
    # Evaluate geometry at one yaw
    # ============================================================

    def evaluate_angle(
        self,
        xy,
        yaw
    ):

        c = math.cos(yaw)
        s = math.sin(yaw)

        long_axis = np.array(
            [c, s],
            dtype=np.float64
        )

        short_axis = np.array(
            [-s, c],
            dtype=np.float64
        )

        long_projection = (
            xy @ long_axis
        )

        short_projection = (
            xy @ short_axis
        )

        # Robust boundaries
        long_min = float(
            np.percentile(
                long_projection,
                2
            )
        )

        long_max = float(
            np.percentile(
                long_projection,
                98
            )
        )

        short_min = float(
            np.percentile(
                short_projection,
                2
            )
        )

        short_max = float(
            np.percentile(
                short_projection,
                98
            )
        )

        observed_long = (
            long_max - long_min
        )

        observed_short = (
            short_max - short_min
        )

        # Partial visibility is acceptable.
        # Oversized dimensions are punished more strongly.
        long_missing = max(
            0.0,
            TABLE_LENGTH - observed_long
        )

        short_missing = max(
            0.0,
            TABLE_WIDTH - observed_short
        )

        long_excess = max(
            0.0,
            observed_long - TABLE_LENGTH
        )

        short_excess = max(
            0.0,
            observed_short - TABLE_WIDTH
        )

        geometry_score = (
            long_missing * 0.5
            +
            short_missing * 0.7
            +
            long_excess * 4.0
            +
            short_excess * 5.0
        )

        return {
            'yaw':
                yaw,

            'long_axis':
                long_axis,

            'short_axis':
                short_axis,

            'long_min':
                long_min,

            'long_max':
                long_max,

            'short_min':
                short_min,

            'short_max':
                short_max,

            'observed_long':
                observed_long,

            'observed_short':
                observed_short,

            'geometry_score':
                geometry_score
        }

    # ============================================================
    # Convex-hull edge extraction
    # ============================================================

    def get_hull_edges(
        self,
        cluster
    ):

        xy = cluster[:, :2]

        try:

            hull = ConvexHull(xy)

        except Exception:

            return []

        vertices = xy[
            hull.vertices
        ]

        edges = []

        count = len(vertices)

        for i in range(count):

            p1 = vertices[i]
            p2 = vertices[
                (i + 1) % count
            ]

            vector = (
                p2 - p1
            )

            length = float(
                np.linalg.norm(vector)
            )

            # Ignore tiny hull segments caused by noise.
            if length < 0.07:
                continue

            angle = math.atan2(
                vector[1],
                vector[0]
            )

            angle = self.normalize_yaw(
                angle
            )

            edges.append({
                'angle':
                    angle,

                'length':
                    length
            })

        return edges

    # ============================================================
    # Edge-alignment score
    #
    # Correct rectangle yaw should make hull edges parallel to
    # either:
    #
    #     yaw
    #
    # or
    #
    #     yaw + 90 degrees
    #
    # Long hull edges receive more weight.
    # ============================================================

    def edge_alignment_score(
        self,
        edges,
        yaw
    ):

        if len(edges) == 0:
            return float('inf')

        weighted_error = 0.0
        total_weight = 0.0

        for edge in edges:

            difference = (
                self.angle_difference_90(
                    edge['angle'],
                    yaw
                )
            )

            weight = (
                edge['length']
                ** 2
            )

            weighted_error += (
                difference
                *
                weight
            )

            total_weight += weight

        if total_weight <= 0.0:
            return float('inf')

        return (
            weighted_error
            /
            total_weight
        )

    # ============================================================
    # Find orientation from real outer edges
    # ============================================================

    def find_edge_orientation(
        self,
        cluster
    ):

        xy = cluster[:, :2]

        edges = self.get_hull_edges(
            cluster
        )

        if len(edges) == 0:
            return None

        candidate_angles = []

        # --------------------------------------------------------
        # Each sufficiently long hull edge provides a yaw
        # hypothesis.
        # --------------------------------------------------------

        for edge in edges:

            if edge['length'] < 0.10:
                continue

            angle = edge['angle']

            candidate_angles.append(
                angle
            )

            candidate_angles.append(
                self.normalize_yaw(
                    angle + math.pi / 2.0
                )
            )

        if len(candidate_angles) == 0:
            return None

        best = None
        best_score = float('inf')

        # --------------------------------------------------------
        # Test each hull-derived orientation and refine nearby.
        # --------------------------------------------------------

        for candidate in candidate_angles:

            candidate_deg = math.degrees(
                candidate
            )

            for angle_deg in np.arange(
                candidate_deg - 3.0,
                candidate_deg + 3.01,
                0.25
            ):

                yaw = self.normalize_yaw(
                    math.radians(
                        angle_deg
                    )
                )

                result = self.evaluate_angle(
                    xy,
                    yaw
                )

                edge_score = (
                    self.edge_alignment_score(
                        edges,
                        yaw
                    )
                )

                # Convert angular error into a comparable metric.
                edge_score_deg = math.degrees(
                    edge_score
                )

                total_score = (
                    result[
                        'geometry_score'
                    ]
                    +
                    edge_score_deg
                    * 0.035
                )

                if total_score < best_score:

                    best_score = (
                        total_score
                    )

                    result[
                        'edge_score_deg'
                    ] = edge_score_deg

                    result[
                        'total_score'
                    ] = total_score

                    best = result

        if best is None:
            return None

        # --------------------------------------------------------
        # Ensure the 1.6 m dimension is represented as LONG.
        # --------------------------------------------------------

        if (
            abs(
                best[
                    'observed_short'
                ]
                -
                TABLE_LENGTH
            )
            <
            abs(
                best[
                    'observed_long'
                ]
                -
                TABLE_LENGTH
            )
        ):

            rotated_yaw = (
                best['yaw']
                +
                math.pi / 2.0
            )

            rotated_yaw = self.normalize_yaw(
                rotated_yaw
            )

            rotated = self.evaluate_angle(
                xy,
                rotated_yaw
            )

            rotated[
                'edge_score_deg'
            ] = self.edge_alignment_score(
                edges,
                rotated_yaw
            )

            rotated[
                'edge_score_deg'
            ] = math.degrees(
                rotated[
                    'edge_score_deg'
                ]
            )

            rotated[
                'total_score'
            ] = (
                rotated[
                    'geometry_score'
                ]
                +
                rotated[
                    'edge_score_deg'
                ]
                * 0.035
            )

            best = rotated

        return best

    # ============================================================
    # Center reconstruction
    #
    # Keep the method that produced:
    #
    #   ~1.79, -1.02
    #   ~1.78,  1.69
    #
    # in the previous test.
    # ============================================================

    def reconstruct_axis(
        self,
        minimum,
        maximum,
        physical_size
    ):

        observed_size = (
            maximum - minimum
        )

        visibility_ratio = (
            observed_size
            /
            physical_size
        )

        # Almost completely visible.
        if visibility_ratio >= 0.96:

            return float(
                (
                    minimum
                    +
                    maximum
                )
                /
                2.0
            )

        # Determine which projected edge is closer to sensor
        # origin.
        min_distance = abs(
            minimum
        )

        max_distance = abs(
            maximum
        )

        midpoint = (
            minimum + maximum
        ) / 2.0

        if min_distance <= max_distance:

            direction = np.sign(
                midpoint - minimum
            )

            if direction == 0:
                direction = 1.0

            center = (
                minimum
                +
                direction
                *
                physical_size
                /
                2.0
            )

        else:

            direction = np.sign(
                midpoint - maximum
            )

            if direction == 0:
                direction = -1.0

            center = (
                maximum
                +
                direction
                *
                physical_size
                /
                2.0
            )

        return float(center)

    def reconstruct_center(
        self,
        fit
    ):

        long_center = (
            self.reconstruct_axis(
                fit['long_min'],
                fit['long_max'],
                TABLE_LENGTH
            )
        )

        short_center = (
            self.reconstruct_axis(
                fit['short_min'],
                fit['short_max'],
                TABLE_WIDTH
            )
        )

        center = (
            fit['long_axis']
            *
            long_center
            +
            fit['short_axis']
            *
            short_center
        )

        return center

    # ============================================================
    # Build full physical table rectangle
    # ============================================================

    def build_corners(
        self,
        center,
        yaw
    ):

        c = math.cos(yaw)
        s = math.sin(yaw)

        long_axis = np.array(
            [c, s]
        )

        short_axis = np.array(
            [-s, c]
        )

        half_length = (
            TABLE_LENGTH / 2.0
        )

        half_width = (
            TABLE_WIDTH / 2.0
        )

        return [

            center
            +
            long_axis * half_length
            +
            short_axis * half_width,

            center
            +
            long_axis * half_length
            -
            short_axis * half_width,

            center
            -
            long_axis * half_length
            -
            short_axis * half_width,

            center
            -
            long_axis * half_length
            +
            short_axis * half_width
        ]

    # ============================================================
    # RViz marker
    # ============================================================

    def create_marker(
        self,
        marker_id,
        center,
        yaw,
        z,
        stamp
    ):

        marker = Marker()

        marker.header.frame_id = (
            'base_link'
        )

        marker.header.stamp = stamp

        marker.ns = (
            'detected_tables'
        )

        marker.id = (
            marker_id
        )

        marker.type = (
            Marker.LINE_STRIP
        )

        marker.action = (
            Marker.ADD
        )

        marker.scale.x = 0.035

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        corners = (
            self.build_corners(
                center,
                yaw
            )
        )

        for corner in corners:

            point = Point()

            point.x = float(
                corner[0]
            )

            point.y = float(
                corner[1]
            )

            point.z = float(z)

            marker.points.append(
                point
            )

        first = Point()

        first.x = float(
            corners[0][0]
        )

        first.y = float(
            corners[0][1]
        )

        first.z = float(z)

        marker.points.append(
            first
        )

        return marker

    # ============================================================
    # Main callback
    # ============================================================

    def pointcloud_callback(
        self,
        msg
    ):

        cloud = pc2.read_points(
            msg,
            field_names=(
                'x',
                'y',
                'z'
            ),
            skip_nans=True
        )

        points = np.column_stack([
            cloud['x'],
            cloud['y'],
            cloud['z']
        ]).astype(
            np.float32
        )

        if len(points) == 0:
            return

        # ========================================================
        # 1. Table-height filter
        # ========================================================

        points = points[
            (points[:, 2] >= 0.45)
            &
            (points[:, 2] <= 0.70)
        ]

        if len(points) == 0:
            return

        # ========================================================
        # 2. Range filter
        # ========================================================

        ranges = np.linalg.norm(
            points[:, :2],
            axis=1
        )

        points = points[
            ranges
            <=
            MAX_DETECTION_RANGE
        ]
        # ========================================================
        # Remove long wall structures BEFORE table clustering
        # ========================================================

        points = self.remove_wall_lines(
            points
        )

        if len(points) == 0:
            return

        # ========================================================
        # 3. Clustering
        # ========================================================

        clusters = self.cluster_xy(
            points,
            radius=0.25,
            min_points=25
        )

        detections = []

        # ========================================================
        # 4. Analyze every cluster independently
        # ========================================================

        for cluster in clusters:

            (
                z_mean,
                z_std,
                robust_z_span,
                hull_area
            ) = self.surface_statistics(
                cluster
            )

            # ----------------------------------------------------
            # Table height
            # ----------------------------------------------------

            if not (
                0.52
                <=
                z_mean
                <=
                0.61
            ):
                continue

            # ----------------------------------------------------
            # Reject vertical structures / walls
            # ----------------------------------------------------

            if z_std > 0.045:
                continue

            if robust_z_span > 0.18:
                continue

            if len(cluster) < 40:
                continue

            if hull_area < 0.08:
                continue

            # ====================================================
            # Edge-based orientation
            # ====================================================

            fit = (
                self.find_edge_orientation(
                    cluster
                )
            )

            if fit is None:
                continue

            observed_long = (
                fit['observed_long']
            )

            observed_short = (
                fit['observed_short']
            )

            # ====================================================
            # Table-size validation
            # ====================================================

            if not (
                0.90
                <=
                observed_long
                <=
                1.75
            ):
                continue

            if not (
                0.50
                <=
                observed_short
                <=
                0.95
            ):
                continue

            # ====================================================
            # Reconstruct center
            # ====================================================

            center = (
                self.reconstruct_center(
                    fit
                )
            )

            yaw = (
                self.normalize_yaw(
                    fit['yaw']
                )
            )

            detections.append({

                'points':
                    cluster,

                'center':
                    center,

                'yaw':
                    yaw,

                'z':
                    z_mean,

                'z_std':
                    z_std,

                'observed_long':
                    observed_long,

                'observed_short':
                    observed_short,

                'edge_score':
                    fit.get(
                        'edge_score_deg',
                        0.0
                    )
            })

        # ========================================================
        # Clear previous markers
        # ========================================================

        marker_array = MarkerArray()

        clear_marker = Marker()

        clear_marker.action = (
            Marker.DELETEALL
        )

        marker_array.markers.append(
            clear_marker
        )

        # ========================================================
        # No detections
        # ========================================================

        if len(detections) == 0:

            self.markers_pub.publish(
                marker_array
            )

            self.get_logger().warn(
                'Tables detected: 0'
            )

            return

        # Stable IDs only.
        detections.sort(
            key=lambda table: (
                table['center'][1],
                table['center'][0]
            )
        )

        # ========================================================
        # Publish measured table points
        # ========================================================

        combined = np.vstack([
            table['points']
            for table in detections
        ])

        header = msg.header

        header.frame_id = (
            'base_link'
        )

        detected_cloud = (
            pc2.create_cloud_xyz32(
                header,
                combined
            )
        )

        self.tables_pub.publish(
            detected_cloud
        )

        # ========================================================
        # PoseArray
        # ========================================================

        pose_array = PoseArray()

        pose_array.header = msg.header

        pose_array.header.frame_id = (
            'base_link'
        )

        # ========================================================
        # Markers + console
        # ========================================================

        for table_id, table in enumerate(
            detections
        ):

            center = (
                table['center']
            )

            yaw = (
                table['yaw']
            )

            pose = Pose()

            pose.position.x = float(
                center[0]
            )

            pose.position.y = float(
                center[1]
            )

            pose.position.z = float(
                table['z']
            )

            pose.orientation.z = float(
                math.sin(
                    yaw / 2.0
                )
            )

            pose.orientation.w = float(
                math.cos(
                    yaw / 2.0
                )
            )

            pose_array.poses.append(
                pose
            )

            marker = (
                self.create_marker(
                    table_id,
                    center,
                    yaw,
                    table['z'],
                    msg.header.stamp
                )
            )

            marker_array.markers.append(
                marker
            )

            self.get_logger().info(

                f'TABLE {table_id} | '

                f'Center '
                f'[{center[0]:.2f}, '
                f'{center[1]:.2f}, '
                f'{table["z"]:.2f}] m | '

                f'Yaw '
                f'{math.degrees(yaw):.1f} deg | '

                f'Observed '
                f'[{table["observed_long"]:.2f} x '
                f'{table["observed_short"]:.2f}] m | '

                f'EdgeErr '
                f'{table["edge_score"]:.1f} deg | '

                f'Zstd '
                f'{table["z_std"]:.3f}'
            )

        self.centers_pub.publish(
            pose_array
        )

        self.markers_pub.publish(
            marker_array
        )

        self.get_logger().info(
            f'Tables detected: '
            f'{len(detections)}'
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = TableDetector()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()