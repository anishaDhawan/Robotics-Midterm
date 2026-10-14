# TurtleBot3 Waffle Mission Manager — Oct 16 Midterm

This is **the ROS2 Python orchestration package**, not a complete standalone robotics solution.
The `MissionManager` node subscribes to `goal1`, `goal2`, `goal3` and an optional
human-location result from a **separate OpenCV node**. It sends navigation requests
to Nav2's `NavigateToPose` action and publishes temporary JSON status strings.

## Scope and current completion

| Requirement | This package | Needs a teammate / integration |
|---|---|---|
| `/goal1` before `/goal2` | Implemented and pure-Python tested | Nav2 and correct map frame |
| `/goal3` interrupts active goal | Requests Nav2 action cancellation; emergency has priority | Real Nav2 cancellation and timing validation |
| Resume prior goal after `/goal3` | Implemented and pure-Python tested | Real Nav2 validation |
| `/status` updates | Yes: `std_msgs/String` carrying JSON | **Professor's specified message type is pending** |
| Human location report / stop mission | Yes, on receiving `human_location` PointStamped | **OpenCV detection AND map-coordinate estimation not included** |
| Search for human | Optional repeated Nav2 waypoint patrol *after goals* | Choose reachable waypoints or integrate a real exploration planner |
| Obstacles / robots | Delegates to Nav2 | Teammate Nav2 costmaps, LiDAR, camera safety |
| 30-second performance, 1m tolerance | Not guaranteed by the Python node | Tune Nav2, test Gazebo ground truth |
| Team namespace | Relative ROS2 topic/action names | Confirm actual simulator and grader mapping |

**Do not claim the full midterm requirements are satisfied simply because unit tests pass.**

## Latest instructor update (Oct 13): operational considerations

- **The professor supplies goals** by publishing `geometry_msgs/Point` on
  `/goal1`, `/goal2`, and `/goal3` at evaluation time. Do not hardcode any
  destination values. Manual publish commands below are *local tests only*.
- **All goal coordinates are in `map` frame**, which is the default
  `frame_id` parameter for Nav2 requests. This is not the `odom` frame.
- **Goals may be unexplored.** The mission node does *not* magically map an
  unexplored room. Person 1 must configure SLAM and Nav2 planner/costmaps
  (e.g., known vs unknown traversal), or provide an exploration/recovery
  strategy. Verify that unknown goals can actually be reached in House world.
- **`/odom` may drift** and the evaluator checks **Gazebo ground truth**.
  Nav2's reported success is not ground truth. Tune navigation/localization
  and aim safely within the 1 m boundary (e.g. configure an XY tolerance
  stricter than 1 m, allowing for localization error).
- **The professor scores the recorded time sequence of `/status`.** The
  mission node publishes `GOAL1_REACHED`, `GOAL2_REACHED`, or
  `EMERGENCY_REACHED` immediately after the corresponding Nav2 action reports
  success. Its temporary JSON messages include `elapsed_s` and
  `within_30s` measured from receipt with a monotonic clock for *diagnostics*.
  A one-time warning is logged if an outstanding goal exceeds 30 seconds;
  the node does not artificially declare success.
- **Human victim reporting** must include the *human's map-frame coordinates*
  accurate to 1 m, not just the robot's own position. Our node can relay a
  teammate's `PointStamped`, but real OpenCV detection, image-to-map geometry,
  and active scene search remain unimplemented here.
- **Status message type is still pending.** A JSON `std_msgs/String` is only a
  developer placeholder; the grader will likely require the professor's
  specific ROS2 message type. Do not assume the current message is accepted.

### Where to tune goal acceptance

Nav2 Humble provides `SimpleGoalChecker` with `xy_goal_tolerance` and
`yaw_goal_tolerance` as configurable parameters. Discuss a position-focused
checker with Person 1 since the professor gives only `(x, y)` and doesn't
score final yaw. The starter sends an identity orientation (yaw=0), which
could waste time turning at the destination until Nav2's goal checker is tuned.

- Humble goal checker: https://api.nav2.org/nav2-humble/html/classnav2__controller_1_1SimpleGoalChecker.html
- NavFn planner can be configured to allow unknown space: https://api.nav2.org/nav2-humble/html/classnav2__navfn__planner_1_1NavFn.html

## File explanations

- `team_mission/mission_logic.py`: Pure Python. Stores the two ordinary goals,
  emergency and human-found flags. Chooses priority: human detection (terminal)
  > emergency Goal3 > Goal1 > Goal2 > optional map waypoint search.
- `team_mission/mission_manager.py`: ROS2 `rclpy` node. Receives topics; builds
  Nav2 `NavigateToPose` goals; asynchronously cancels for emergencies; receives
  success, canceled and failure results; emits status messages.
- `test/test_mission_logic.py`: Unit tests independent of ROS2.
- `setup.py`, `setup.cfg`, `package.xml`, `resource/team_mission`: Standard
  ROS2 Python package metadata.

## Assumptions you MUST confirm

1. Goal topics use **`geometry_msgs/msg/Point`**. Goal3 uses the same type,
   pending instructor confirmation.
2. All x/y goals are in the Nav2 map frame by default (`frame_id: map`).
3. Nav2's `NavigateToPose` action server exists and is configured for this
   TurtleBot3 robot / namespace. This node does not publish wheel velocities.
4. `status` is **temporarily `std_msgs/msg/String` with JSON content**, because
   the instructor said the actual message type will be provided later. Once
   announced, replace `_status()` in `mission_manager.py` and its publisher.
5. This is **one two-goal mission per process**: the first valid message on
   each normal goal topic wins. Subsequent messages are ignored. Emergency
   Goal3 can be sent again; new distinct emergency positions override older ones.
6. `human_location` is a **teammate-defined interface**, not professor-provided:
   `geometry_msgs/msg/PointStamped`, with `header.frame_id="map"` and
   `point.x/y` being the detected human's map coordinates. The OpenCV component
   must avoid false detections and transform image observations to map frame.
7. After reaching Goal2, the robot **must continue looking for a human**.
   Supply valid `search_waypoints_xy` for basic coverage, or connect an actual
   SLAM/frontier exploration algorithm. Patrol cannot guarantee coverage.
8. A human-found message is treated as definitive and terminal. A false
   positive would incorrectly stop the mission.
9. Emergency cancellation is asynchronous; a Nav2 cancellation that fails or
   never produces an action result can block dispatch. Test interrupt latency.
10. Nav2 action success is **not the same as Gazebo ground-truth position within
    1m**; test and tune localization and goal-checker settings.

## Install and test locally

ROS2 Humble and a working Python3 `colcon` environment are assumed.

Place this directory at `<workspace>/src/team_mission/`.
From package directory, run pure logic tests:

```bash
python3 -m unittest discover -s test -v
```

From the ROS2 workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select team_mission --symlink-install
source install/setup.bash
ros2 pkg executables team_mission
```

Expected executable: `team_mission mission_manager`.

## Run

Start Gazebo, TurtleBot3 Waffle, Nav2, mapping/localization, and safety nodes
according to the **rest of your team's project**. Then (in a sourced shell):

```bash
ros2 run team_mission mission_manager --ros-args -r __ns:=/team1 -p use_sim_time:=true
```

Default relative topics resolve to:

- `/goal1` — professor-provided `geometry_msgs/msg/Point` (no coordinates hardcoded)
- `/goal2` — professor-provided `geometry_msgs/msg/Point`
- `/goal3` — professor-provided emergency `geometry_msgs/msg/Point`
- `/status` — **TEMPORARY** `std_msgs/msg/String` JSON (instructor's exact message type pending)
- `/team1/human_location` — teammate-provided `geometry_msgs/msg/PointStamped`
- `/team1/navigate_to_pose` — Nav2 action name

**Namespace ambiguity:** The assignment requires team-prefixed topics, but
explicitly calls the grading interface `/goal1`, `/goal2`, `/goal3`, `/status`.
This starter defaults to those four literal root names so the professor's
publisher/recorder can connect. The robot/Nav2/vision interfaces remain
namespaced. Ask the instructor to confirm whether the grader uses root
names or `/team1/*` names; switch via ROS parameters without editing code.

> If the professor instead expects namespaced evaluation topics,
> run the following (no code changes required):

```bash
ros2 run team_mission mission_manager --ros-args -r __ns:=/team1 \
  -p use_sim_time:=true \
  -p goal1_topic:=goal1 -p goal2_topic:=goal2 \
  -p goal3_topic:=goal3 -p status_topic:=status
```

> If Person 1's Nav2 action is global, also supply
> `-p nav2_action:=/navigate_to_pose`.

Check the actual interfaces with:

```bash
ros2 topic list -t
ros2 action list -t
ros2 node info /team1/mission_manager
```

### Optional waypoint search after Goal2

For a **known map**, you can pass reachable `(x,y)` patrol points (in map frame).
After both normal goals succeed, Nav2 repeatedly cycles through them, seeking
human detections. This provides *basic search motion*, **not** coverage planning
or guaranteed discovery. The emergency goal can interrupt this search; on
return, the same waypoint is resumed.

```bash
ros2 run team_mission mission_manager --ros-args -r __ns:=/team1 \
  -p use_sim_time:=true \
  -p 'search_waypoints_xy:=1.0,2.0;2.5,3.0;0.0,1.0'
```

**Only use waypoints your Nav2 teammate confirms are reachable and appropriate
for the evaluation world.** Default is no patrol; after Goal2 it waits for a
human report.

## Manual evaluation scenarios (each on a fresh node run)

From a second ROS2-sourced terminal, after Nav2 is active:

```bash
ros2 topic echo /status std_msgs/msg/String
```

Publish Goal2 first; the robot must not navigate to it yet:

```bash
ros2 topic pub --once /goal2 geometry_msgs/msg/Point '{x: 1.0, y: 1.0, z: 0.0}'
```

Publish Goal1; it should navigate to Goal1 first, then Goal2:

```bash
ros2 topic pub --once /goal1 geometry_msgs/msg/Point '{x: 0.0, y: 0.0, z: 0.0}'
```

While moving, publish emergency Goal3. The node should report emergency,
request cancellation, navigate to Goal3, and then resume unfinished Goal1/2:

```bash
ros2 topic pub --once /goal3 geometry_msgs/msg/Point '{x: 0.5, y: 0.5, z: 0.0}'
```

Finally **TEST THE HANDOFF**, simulating a confirmed vision result using a map
coordinate that you chose. This does *not* test real OpenCV detection:

```bash
ros2 topic pub --once /team1/human_location geometry_msgs/msg/PointStamped \
  '{header: {frame_id: "map"}, point: {x: 0.3, y: 1.6, z: 0.0}}'
```

The node publishes a `HUMAN_FOUND` JSON event containing `x`, `y`, and
`frame_id`, then requests cancellation of ongoing Nav2 navigation. No further
goals should be dispatched. Do not send fake human detections in a graded run.

These x/y values are **illustrative only**. Pick reachable goals for the actual
map; goal arrival cannot be tested without Nav2 and Gazebo.

### Example JSON status strings

```json
{"event":"GOAL1_REACHED","state":"READY_GOAL2","x":1.0,"y":2.0}
{"event":"EMERGENCY_HANDLING","state":"EMERGENCY_HANDLING","x":3.0,"y":4.0}
{"event":"EMERGENCY_REACHED","state":"READY_GOAL1","x":3.0,"y":4.0}
{"event":"HUMAN_FOUND","frame_id":"map","state":"HUMAN_FOUND","x":2.4,"y":1.8}
```

## Interface contract for teammates

**Person 1 (Nav2):** Supply `/team1/navigate_to_pose` (or tell us the actual
action name), correct TF/map frame, localization, LiDAR costmaps and tuning for
rapid arrivals. The ROS2 mission node uses this action, not direct `cmd_vel`.

**Person 3 (OpenCV):** Implement actual multi-pose human detector (standing,
sitting, lying), identify a human from a camera frame and estimate the human's
map coordinate using calibration/depth or another grounded approach. Publish
`PointStamped` on `/team1/human_location`. A standard upright-only HOG detector
is **not sufficient** for all the required poses.

**Person 4 (safety/integration):** Ensure single ownership of final
`/team1/cmd_vel`; coordinate Nav2 with dynamic-obstacle sensing and emergency
cancellation. Make sure multiple ROS2 nodes do not issue competing velocities.

## Limitations and priorities

- This implementation waits for the Nav2 cancellation result before sending
  the emergency action. Test and fix any unacceptable cancellation delay or
  canceled-result race in your *actual* Nav2 setup, especially the 30s rubric.
- When a normal Nav2 action fails, this starter reports failure instead of
  automatically retrying. Retrying or recovering from false aborts may improve
  the evaluation result.
- The requested yaw is zero radians (`orientation.w = 1`) since professor
  supplies only `(x,y)`. Nav2's yaw tolerance and map-frame interpretation may
  need tuning to meet 30-second/1-meter ground-truth thresholds.
- Changing the `/status` type is compulsory when the instructor specifies it.
- ROS2/Gazebo execution could not be validated in the build environment where
  this starter code was packaged; only pure Python unit tests and syntax checks
  were run.

## Sources & acknowledgement

- ROS2 Humble: https://docs.ros.org/en/humble/
- Nav2 Humble `NavigateToPose`: https://api.nav2.org/actions/humble/navigatetopose.html
- ROS2 Navigation2 repository: https://github.com/ros-navigation/navigation2
- The mission-manager code was drafted with AI assistance. Review and test it,
  add citations for any additional algorithms/models, and verify against your
  course's academic integrity rules before submission.

## Git workflow (when *you* decide to push)

1. Work in your own branch (suggested `feature/goal-manager`).
2. Run tests and a ROS2 build in your own Ubuntu environment.
3. Confirm README assumptions with teammates.
4. Stage and commit files, then push your branch when ready; do **not** push
   straight to `main` without your team's review/merge plan.
