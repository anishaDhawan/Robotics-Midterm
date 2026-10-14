"""Pure Python mission policy for ROS2 Humble's TurtleBot3 midterm project.

Priority: human found (terminal) > emergency goal3 > goal1 > goal2.
Goal3 interrupts navigation; canceled normal goals are still pending and resume.
No ROS imports: tests can run on a laptop without Gazebo or ROS2.
"""

from dataclasses import dataclass
from math import isfinite
from typing import Optional, Tuple


@dataclass(frozen=True)
class Target:
    kind: str                 # 'goal1', 'goal2', 'goal3', or 'search'
    x: float
    y: float
    generation: int = 0      # distinguishes consecutive emergencies

    def __post_init__(self):
        if self.kind not in ('goal1', 'goal2', 'goal3', 'search'):
            raise ValueError('Unknown target kind')
        if not isfinite(self.x) or not isfinite(self.y):
            raise ValueError('Goal x and y must be finite')


class MissionLogic:
    """Deterministic navigation-order rules; Nav2 handles actual driving."""

    def __init__(self):
        self.goal1: Optional[Target] = None
        self.goal2: Optional[Target] = None
        self.emergency: Optional[Target] = None
        self.active: Optional[Target] = None
        self.goal1_reached = False
        self.goal2_reached = False
        self.human: Optional[Tuple[float, float]] = None
        self.normal_failure: Optional[str] = None
        self._emergency_number = 0
        self.search_waypoints = []
        self.search_index = 0

    def receive_normal_goal(self, number: int, x: float, y: float) -> bool:
        """Store first message on goal1/goal2; duplicates are ignored."""
        if number not in (1, 2):
            raise ValueError('Normal goal number must be 1 or 2')
        candidate = Target(f'goal{number}', float(x), float(y))
        existing = self.goal1 if number == 1 else self.goal2
        if self.human is not None or existing is not None:
            return False
        if number == 1:
            self.goal1 = candidate
        else:
            self.goal2 = candidate
        return True

    def receive_emergency(self, x: float, y: float) -> bool:
        """Goal3 can arrive at any time. Most recent *different* target wins.

        Repeated identical messages while an emergency is pending are ignored.
        Once the emergency succeeds, a new message can trigger a new emergency.
        """
        candidate = Target('goal3', float(x), float(y))
        if self.human is not None:
            return False
        if (self.emergency is not None and
                (self.emergency.x, self.emergency.y) == (candidate.x, candidate.y)):
            return False
        self._emergency_number += 1
        self.emergency = Target('goal3', candidate.x, candidate.y,
                                self._emergency_number)
        return True

    def configure_search(self, waypoints):
        """Optional map-frame patrol waypoints after both normal goals succeed."""
        self.search_waypoints = [
            Target('search', float(x), float(y), generation=i)
            for i, (x, y) in enumerate(waypoints)
        ]
        self.search_index = 0

    def report_human(self, x: float, y: float) -> bool:
        """A validated map-frame detection ends the mission permanently."""
        x, y = float(x), float(y)
        if not (isfinite(x) and isfinite(y)):
            raise ValueError('Human map coordinates must be finite')
        if self.human is not None:
            return False
        self.human = (x, y)
        return True

    def needs_interrupt(self) -> bool:
        if self.active is None:
            return False
        if self.human is not None:
            return True
        return self.emergency is not None and self.emergency != self.active

    def next_target(self) -> Optional[Target]:
        """Select next Nav2 target, but only when no task is currently active."""
        if self.human is not None or self.active is not None:
            return None
        if self.emergency is not None:
            return self.emergency
        if self.normal_failure is not None:
            return None
        if not self.goal1_reached:
            return self.goal1
        if not self.goal2_reached:
            return self.goal2
        if self.search_waypoints:
            return self.search_waypoints[self.search_index]
        return None

    def start(self, target: Target) -> None:
        if self.next_target() != target:
            raise ValueError('Attempt to start an out-of-order task')
        self.active = target

    def finish(self, target: Target, outcome: str) -> None:
        """outcome: succeeded, canceled (preempted), or failed."""
        if self.active != target:
            raise ValueError('Result does not match active navigation')
        if outcome not in ('succeeded', 'canceled', 'failed'):
            raise ValueError('Invalid outcome')
        self.active = None
        if outcome == 'canceled':
            # A canceled regular goal remains unfinished and will be retried.
            return
        if outcome == 'succeeded':
            if target.kind == 'goal1':
                self.goal1_reached = True
            elif target.kind == 'goal2':
                self.goal2_reached = True
            elif self.emergency == target:
                self.emergency = None
            elif target.kind == 'search' and self.search_waypoints:
                self.search_index = (self.search_index + 1) % len(self.search_waypoints)
            return
        if target.kind == 'goal3':
            # Don't permanently block Goal1/Goal2 if Nav2 rejects Goal3.
            # Alert the operator; this emergency remains unfulfilled.
            if self.emergency == target:
                self.emergency = None
        elif target.kind == 'search' and self.search_waypoints:
            # Skip an unreachable patrol point instead of blocking the mission.
            self.search_index = (self.search_index + 1) % len(self.search_waypoints)
        else:
            self.normal_failure = target.kind

    @property
    def state(self) -> str:
        if self.human is not None:
            return 'HUMAN_FOUND'
        if self.emergency is not None:
            return 'EMERGENCY_HANDLING'
        if self.active is not None:
            return f'NAVIGATING_{self.active.kind.upper()}'
        if self.normal_failure is not None:
            return 'NAVIGATION_FAILED'
        if not self.goal1_reached:
            return 'WAITING_FOR_GOAL1' if self.goal1 is None else 'READY_GOAL1'
        if not self.goal2_reached:
            return 'WAITING_FOR_GOAL2' if self.goal2 is None else 'READY_GOAL2'
        return 'SEARCHING_FOR_HUMAN' if self.search_waypoints else 'GOALS_REACHED_AWAITING_HUMAN'
