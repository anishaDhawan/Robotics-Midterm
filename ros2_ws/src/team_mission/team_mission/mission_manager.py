"""ROS2 Humble goal manager: goal1 -> goal2; goal3 interrupts and resumes.

Human detection is an interface: teammate's OpenCV node should publish a
geometry_msgs/PointStamped in the *map* frame on `human_location`.

Temporary /status contract: std_msgs/String containing JSON. Instructor has
not yet specified the final status message type; update this publisher later.

Nav2 action reference: https://api.nav2.org/actions/humble/navigatetopose.html
"""

import json
import math
import time
from typing import Any, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from team_mission.mission_logic import MissionLogic, Target


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        # The assignment names these *grader-facing* topics at the root.
        # ROS robot/vision internals stay in the team's namespace. If the
        # instructor instead requires /team1/goal1 and /team1/status, override
        # these four parameters. No goal coordinates are hardcoded.
        self.declare_parameter('goal1_topic', '/goal1')
        self.declare_parameter('goal2_topic', '/goal2')
        self.declare_parameter('goal3_topic', '/goal3')
        self.declare_parameter('status_topic', '/status')
        self.declare_parameter('human_topic', 'human_location')
        self.declare_parameter('nav2_action', 'navigate_to_pose')
        self.declare_parameter('frame_id', 'map')
        # Optional semicolon-separated x,y map waypoints for human search.
        # Example: '1.0,2.0;3.0,2.5' (empty means no patrol).
        self.declare_parameter('search_waypoints_xy', '')

        self.logic = MissionLogic()
        waypoint_text = self.get_parameter('search_waypoints_xy').value.strip()
        if waypoint_text:
            try:
                waypoints = []
                for pair in waypoint_text.split(';'):
                    x_text, y_text = pair.split(',')
                    waypoints.append((float(x_text.strip()), float(y_text.strip())))
                self.logic.configure_search(waypoints)
            except (ValueError, TypeError) as exc:
                raise ValueError(
                    'search_waypoints_xy must look like 1.0,2.0;3.0,2.5'
                ) from exc
        self._client = ActionClient(
            self, NavigateToPose, self.get_parameter('nav2_action').value)
        self._goal_handle = None
        self._sending = False
        self._canceling = False
        self._interrupt_requested = False
        self._active_token: Optional[int] = None
        self._token_counter = 0
        self._last_server_warning = 0.0
        self._resume_after_emergency = False
        # Monotonic receipt times are for local 30-second diagnostics only;
        # instructor evaluation is based on recorded /status and Gazebo truth.
        self._goal_received_at = {}
        self._deadline_warned = set()

        self.create_subscription(
            Point, self.get_parameter('goal1_topic').value,
            lambda msg: self._receive_goal(1, msg), 10)
        self.create_subscription(
            Point, self.get_parameter('goal2_topic').value,
            lambda msg: self._receive_goal(2, msg), 10)
        self.create_subscription(
            Point, self.get_parameter('goal3_topic').value,
            self._receive_emergency, 10)
        self.create_subscription(
            PointStamped, self.get_parameter('human_topic').value,
            self._receive_human, 10)

        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, qos)
        self.create_timer(0.2, self._tick)
        self._status('READY')
        self.get_logger().info(
            'Mission manager ready. Goal3 has emergency priority. '
            'Listening to /goal1, /goal2, /goal3 by default.')

    def _status(self, event: str, **details):
        """ONE location to replace when the instructor supplies status.msg."""
        payload = {'event': event, 'state': self.logic.state}
        payload.update(details)
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'), sort_keys=True)
        self._status_pub.publish(msg)
        self.get_logger().info(f'/status: {msg.data}')

    @staticmethod
    def _valid_point(msg):
        return math.isfinite(msg.x) and math.isfinite(msg.y)

    def _receive_goal(self, number: int, msg: Point):
        if not self._valid_point(msg):
            self.get_logger().warning(f'Rejecting invalid Goal{number}.')
            return
        if self.logic.receive_normal_goal(number, msg.x, msg.y):
            self._goal_received_at[f'goal{number}'] = time.monotonic()
            self._status(f'GOAL{number}_RECEIVED', x=msg.x, y=msg.y)
            self._tick()
        else:
            self.get_logger().info(f'Ignoring repeated Goal{number}.')

    def _receive_emergency(self, msg: Point):
        if not self._valid_point(msg):
            self.get_logger().warning('Rejecting invalid emergency coordinates.')
            return
        if not self.logic.receive_emergency(msg.x, msg.y):
            self.get_logger().info('Ignoring duplicate emergency target.')
            return
        self._goal_received_at['goal3'] = time.monotonic()
        self._deadline_warned.discard('goal3')
        # Report emergency immediately, BEFORE Nav2 cancel/action completes.
        self._status('EMERGENCY_HANDLING', x=msg.x, y=msg.y)
        self._tick()

    def _receive_human(self, msg: PointStamped):
        frame = self.get_parameter('frame_id').value
        if msg.header.frame_id != frame:
            self.get_logger().error(
                f'Human coordinates must be in {frame!r}; got {msg.header.frame_id!r}. '
                'Vision teammate must transform camera coordinates into map frame.')
            return
        if not self._valid_point(msg.point):
            self.get_logger().warning('Rejecting invalid human coordinates.')
            return
        if self.logic.report_human(msg.point.x, msg.point.y):
            self._status('HUMAN_FOUND', x=msg.point.x, y=msg.point.y,
                         frame_id=frame)
            self._tick()  # terminal state: cancel any ongoing navigation

    def _tick(self):
        """Event-driven priority selection, plus periodic check for Nav2 readiness."""
        self._check_deadline_diagnostics()
        if self.logic.needs_interrupt():
            self._request_interrupt()
            return
        if self.logic.active is not None or self._sending or self._canceling:
            return
        target = self.logic.next_target()
        if target is None:
            return
        if not self._client.server_is_ready():
            now = time.monotonic()
            if now - self._last_server_warning >= 5.0:
                self.get_logger().warning('Waiting for Nav2 NavigateToPose server.')
                self._last_server_warning = now
            return
        self._dispatch(target)

    def _check_deadline_diagnostics(self):
        """Warn once when 30 real seconds pass without reported arrival.

        This does NOT stop or mark a goal complete; the grader checks /status
        timing and Gazebo ground truth independently.
        """
        now = time.monotonic()
        reached = {
            'goal1': self.logic.goal1_reached,
            'goal2': self.logic.goal2_reached,
            'goal3': self.logic.emergency is None,
        }
        for name, started in self._goal_received_at.items():
            if name in self._deadline_warned or reached[name]:
                continue
            elapsed = now - started
            if elapsed > 30.0:
                self._deadline_warned.add(name)
                self.get_logger().warning(
                    f'{name} has not been reached within 30s '
                    f'({elapsed:.1f}s elapsed). Check navigation, exploration, '
                    'and 1m ground-truth accuracy.')

    def _arrival_fields(self, target: Target):
        """Additional temporary JSON status fields for timing diagnostics."""
        started = self._goal_received_at.get(target.kind)
        if started is None:
            return {}
        elapsed = max(0.0, time.monotonic() - started)
        return {
            'elapsed_s': round(elapsed, 3),
            'within_30s': elapsed <= 30.0,
        }

    def _dispatch(self, target: Target):
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = self.get_parameter('frame_id').value
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x = target.x
        request.pose.pose.position.y = target.y
        request.pose.pose.position.z = 0.0
        # Point has no heading. Identity quaternion requests yaw = 0.
        request.pose.pose.orientation.w = 1.0

        self.logic.start(target)
        self._token_counter += 1
        token = self._token_counter
        self._active_token = token
        self._sending = True
        self._goal_handle = None
        self._interrupt_requested = False

        if target.kind in ('goal1', 'goal2', 'search') and self._resume_after_emergency:
            self._status(f'RESUMING_{target.kind.upper()}', x=target.x, y=target.y)
            self._resume_after_emergency = False
        elif target.kind == 'goal3':
            self._status('EMERGENCY_NAVIGATING', x=target.x, y=target.y)
        else:
            self._status(f'NAVIGATING_{target.kind.upper()}', x=target.x, y=target.y)

        try:
            future = self._client.send_goal_async(request)
            future.add_done_callback(
                lambda f, t=target, ident=token: self._on_accept(f, t, ident))
        except Exception as exc:
            self.get_logger().error(f'Sending {target.kind} failed: {exc}')
            self._finish(target, token, 'failed')

    def _on_accept(self, future: Any, target: Target, token: int):
        if token != self._active_token:
            return
        self._sending = False
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self.get_logger().error(f'Nav2 rejected {target.kind}.')
                self._finish(
                    target, token,
                    'canceled' if self.logic.needs_interrupt() else 'failed'
                )
                return
            self._goal_handle = handle
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda f, t=target, ident=token: self._on_result(f, t, ident))
            # If emergency arrived while send_goal_async was pending, cancel now.
            self._tick()
        except Exception as exc:
            self.get_logger().error(f'Nav2 goal acceptance failed: {exc}')
            self._finish(
                target, token,
                'canceled' if self.logic.needs_interrupt() else 'failed'
            )

    def _request_interrupt(self):
        if self.logic.active is None:
            return
        self._interrupt_requested = True
        if self._sending or self._canceling or self._goal_handle is None:
            return
        self._canceling = True
        token = self._active_token
        self.get_logger().warning('Canceling current Nav2 goal due to higher priority.')
        try:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda f, ident=token: self._on_cancel_response(f, ident))
        except Exception as exc:
            self._canceling = False
            self.get_logger().error(f'Unable to request Nav2 cancellation: {exc}')
            self._status('CANCEL_REQUEST_FAILED', error=str(exc))

    def _on_cancel_response(self, future: Any, token: int):
        if token != self._active_token:
            return
        try:
            response = future.result()
            # Actual cancellation is confirmed by the action result callback.
            if not response.goals_canceling:
                self.get_logger().warning('Nav2 did not accept the cancel request.')
                self._status('CANCEL_NOT_ACCEPTED')
        except Exception as exc:
            self.get_logger().error(f'Cancel response error: {exc}')
            self._status('CANCEL_REQUEST_FAILED', error=str(exc))
        # Continue waiting for get_result_async() before dispatching another goal.
        # This avoids racing two simultaneous velocity controllers.

    def _on_result(self, future: Any, target: Target, token: int):
        if token != self._active_token:
            return
        try:
            response = future.result()
            if response.status == GoalStatus.STATUS_SUCCEEDED:
                outcome = 'succeeded'
            elif self._interrupt_requested:
                # Goal may be canceled or aborted during a deliberate preemption.
                # Never count the interrupted normal goal as reached.
                outcome = 'canceled'
            else:
                outcome = 'failed'
        except Exception as exc:
            self.get_logger().error(f'Nav2 result error: {exc}')
            outcome = 'canceled' if self._interrupt_requested else 'failed'
        self._finish(target, token, outcome)

    def _finish(self, target: Target, token: int, outcome: str):
        if token != self._active_token:
            return
        self._goal_handle = None
        self._active_token = None
        self._sending = False
        self._canceling = False
        self._interrupt_requested = False
        # Logic still knows the target that was active.
        self.logic.finish(target, outcome)

        # Human found is terminal. Do not overwrite its latched status when
        # the interrupted Nav2 action eventually reports cancellation.
        if self.logic.human is not None:
            return

        if outcome == 'succeeded':
            if target.kind == 'goal3':
                self._resume_after_emergency = True
                self._status('EMERGENCY_REACHED', x=target.x, y=target.y,
                             **self._arrival_fields(target))
            elif target.kind == 'search':
                self._status('SEARCH_WAYPOINT_REACHED', x=target.x, y=target.y)
            else:
                self._status(f'{target.kind.upper()}_REACHED', x=target.x, y=target.y,
                             **self._arrival_fields(target))
        elif outcome == 'canceled':
            self._status(f'{target.kind.upper()}_INTERRUPTED')
        else:
            self._status('NAVIGATION_FAILED', target=target.kind)
        # Re-dispatch immediately so emergency cancellation doesn't add timer delay.
        self._tick()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
