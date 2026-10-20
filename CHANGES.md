# What was merged (main + olive-vision-minimal)

Base: main (our navigation_launch.py, explore.py, goal1/2/3 logic, global topics).

Added from olive-vision-minimal (unchanged): human_location_node.py, fused_depth.py,
models.py, geometry.py.

Not taken: rgbd_house.launch.py, waffle_rgbd.* , rgbd_bridge.yaml, depth_obstacles.py,
team2 namespacing, olive's navigation/explore rewrites.

Edited:
- navigation.py: subscribes /human_location; first valid map-frame point sets
  human_found + human_location, clears goal queues, cancels the active Nav2 goal,
  pauses exploration, holds zero velocity; later goals/results are ignored.
  Everything else (goal1/2/3 priority logic) is main's, untouched.
- launch/human_location.launch.py: new, global namespace, remappable camera topics.
- launch/midterm.launch.py: also starts human detection (vision:=false to skip).
- setup.py: added missing `explore` entry point (main's midterm launch needs it)
  and `human_location_node`.
- package.xml: added the dependencies the new nodes use.
- README.md: main's steps + human detection setup.
