# Robot Perception Manager

ROS 2 Jazzy — Home Assignment 2. A detection pipeline driven by an action
server, with a runtime-configurable confidence threshold and a static
camera transform.

## Architecture

```
camera_tf_broadcaster ──► static TF: base_link -> camera_optical_frame

perception_manager
 ├── Action server:  /start_detection   (StartDetection.action)
 ├── Service server: /set_confidence    (SetConfidenceThreshold.srv)
 ├── Publisher:      /detections        (Detection.msg, ~10 Hz while a goal runs)
 └── Parameter:      confidence_threshold (default 0.5, also settable via the service)
```

Both nodes run under the `/perception` namespace and read all values from
`config/perception_params.yaml` — nothing is hardcoded.

## Packages

- `perception_interfaces` — the custom msg / srv / action definitions
- `perception_manager` — the two nodes, config and launch file

## Build

```bash
cd ~/ros2_ws/src
git clone git@github.com:moses-maroun/robot_perception_manager.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch perception_manager perception.launch.py
```

## Try it

```bash
# Start detection and watch feedback
ros2 action send_goal /perception/start_detection \
  perception_interfaces/action/StartDetection "{target_class: person}" --feedback

# Change the threshold mid-run (valid) and test validation (rejected)
ros2 service call /perception/set_confidence \
  perception_interfaces/srv/SetConfidenceThreshold "{threshold: 0.8}"
ros2 service call /perception/set_confidence \
  perception_interfaces/srv/SetConfidenceThreshold "{threshold: 1.5}"

# Cancel: Ctrl+C in the send_goal terminal

# Preemption (bonus): send a second goal while one is running —
# the first returns CANCELED and the new one starts immediately.

# Check the TF tree
ros2 run tf2_tools view_frames
```

## Notes

- The action runs until cancelled or preempted; the result reports the
  total number of detections published.
- Detections use the current threshold on every message, so service calls
  take effect immediately.
- The demo recording is in `demo.cast` (play with `asciinema play demo.cast`).