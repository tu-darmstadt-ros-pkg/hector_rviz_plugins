# HectorViewController Playbook Executor

The playbook executor (`execute_viewcontroller_playbook`) drives the [HectorViewController](../README.md#hectorviewcontroller) through a sequence of steps defined in a YAML file by calling its ROS service interface. Use it to script reproducible camera trajectories, e.g. for recordings or demos.

## Usage

Run RViz with a HectorViewController added, then execute a playbook:

```bash
ros2 run hector_rviz_plugins execute_viewcontroller_playbook path/to/playbook.yaml
```

The executor talks to the services under `/<rviz_node_name>/hector_view_controller/...`. The RViz node name defaults to `rviz`. If your RViz node is named differently, override it:

```bash
ros2 run hector_rviz_plugins execute_viewcontroller_playbook path/to/playbook.yaml --ros-args -p rviz_node_name:=my_rviz
```

The executor waits for the view controller services to become available before starting and can be interrupted at any time with `Ctrl+C`.

A complete, commented example is provided in [example-viewcontroller-playbook.yaml](../example-viewcontroller-playbook.yaml).

## Playbook format

A playbook is a YAML list of steps. Each step has an `action` and action-specific keys. Steps run sequentially. Coordinates (`eye`, `focus`) are `[x, y, z]` lists in meters and are interpreted in the step's `frame`. If `frame` is omitted, the frame of the previous coordinate step is reused (initially `map`).

| Action | Keys | Description |
| --- | --- | --- |
| `move_eye` | `eye` (required), `frame`, `disable_animation` (bool), `stop_tracking` (bool), `switch_to_3d_mode` (bool) | Move the camera position, keeping the current focus point. |
| `move_eye_and_focus` | `eye`, `focus` (both required), `frame`, `disable_animation` (bool), `stop_tracking` (bool) | Move the camera position and the point it looks at. |
| `track_frame` | `frame` | Follow a TF frame (e.g. the robot). Pass an empty string (`""`) to stop tracking. |
| `set_view_mode` | `mode` (`"2d"` or `"3d"`), `disable_animation` (bool) | Switch between 3D and top-down 2D orthographic mode. |
| `spin` | `angle` (radians, default `2*pi`), `duration` (s, default `5.0`), `distance`, `z_offset`, `focus`, `frame` | Orbit the camera around the focus point. `distance`/`z_offset` default to the current camera offset; the move is approximated at 30 FPS. |
| `sleep` | `duration` (s) | Pause before the next step. |

`disable_animation` makes the camera jump to the target instead of animating the transition (default `false`). `stop_tracking` detaches the camera from a tracked frame before moving (default `false`).

## Example

```yaml
# Jump to a start pose, then orbit the focus point once over 10 seconds.
- action: move_eye_and_focus
  eye: [80.0, 26.0, 40.0]
  focus: [11.0, -34.0, 0.0]
  frame: "map"
- action: spin
  angle: 6.28
  duration: 10.0
```
