# Sourced by start-mapping.sh / start-localization.sh: choose the scan SLAM Toolbox uses.
#   g2 (default): /scan_slam, deskewed/motion-selected by the physical sensor core.
#   mid360: /scan_mid360_slam, deskewed per point before the existing 0.25-1.20 m height projection.
#           /scan_mid360 is also kept as an uncorrected diagnostic projection.
# Select with ROBOCUP_SLAM_SCAN or "slam_scan" in config.json. Sets SLAM_PARAMS for the caller.
SLAM_SCAN="${ROBOCUP_SLAM_SCAN:-$(python3 -c 'import json, sys
try: print(json.load(open(sys.argv[1])).get("slam_scan", "g2"))
except Exception: print("g2")' "$CONTROL_CENTER_DIR/config.json")}"
SLAM_TEMPLATE_PARAMS="${ROBOCUP_SLAM_PARAMS:-$CONTROL_CENTER_DIR/slam-web.yaml}"
SLAM_PARAMS="$(mktemp /tmp/robocup-slam-XXXXXX.yaml)"
# Bind both default and custom backend profiles before starting any child process.
# The front-end is the only owner of range filtering and movement/time gating.
/usr/bin/python3 "$CONTROL_CENTER_DIR/prepare-slam-profile.py" \
  "$SLAM_TEMPLATE_PARAMS" "$CONTROL_CENTER_DIR/../robot/config/slam_scan.yaml" "$SLAM_SCAN" "$SLAM_PARAMS"
if [ "$SLAM_SCAN" = mid360 ]; then
  ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r __node:=scan_mid360 \
    -r cloud_in:=/livox/lidar -r scan:=/scan_mid360 --params-file "$CONTROL_CENTER_DIR/scan-mid360.yaml" \
    -p "use_sim_time:=${ROBOCUP_USE_SIM_TIME:-false}" &
  /usr/bin/python3 "$CONTROL_CENTER_DIR/../robot/slam_scan.py" --ros-args \
    -r __node:=mid360_slam_scan --params-file "$CONTROL_CENTER_DIR/../robot/config/slam_scan.yaml" \
    -p "use_sim_time:=${ROBOCUP_USE_SIM_TIME:-false}" &
  echo "SLAM scan: /scan_mid360_slam (timed Mid-360S points, height band 0.25-1.20 m)"
elif [ "$SLAM_SCAN" = g2 ]; then
  echo "SLAM scan: /scan_slam (G2 timed points, physical sensor core)"
else
  echo "Unknown SLAM scan source: $SLAM_SCAN (choose g2 or mid360)" >&2
  exit 2
fi
