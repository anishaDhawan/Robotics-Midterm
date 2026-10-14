import unittest

from team_mission.mission_logic import MissionLogic


class TestMissionLogic(unittest.TestCase):
    def setUp(self):
        self.m = MissionLogic()

    def _go(self, target):
        self.m.start(target)
        self.m.finish(target, 'succeeded')

    def test_search_after_two_goals_and_loop(self):
        self.m.configure_search([(3, 4), (5, 6)])
        self.m.receive_normal_goal(1, 1, 1)
        self.m.receive_normal_goal(2, 2, 2)
        self._go(self.m.next_target())
        self._go(self.m.next_target())
        self.assertEqual(self.m.next_target().kind, 'search')
        self.assertEqual(self.m.next_target().x, 3)
        self._go(self.m.next_target())
        self.assertEqual(self.m.next_target().x, 5)
        self._go(self.m.next_target())
        self.assertEqual(self.m.next_target().x, 3)

    def test_emergency_interrupts_search_and_resumes(self):
        self.m.configure_search([(3, 4)])
        self.m.receive_normal_goal(1, 1, 1)
        self.m.receive_normal_goal(2, 2, 2)
        self._go(self.m.next_target())
        self._go(self.m.next_target())
        searching = self.m.next_target()
        self.m.start(searching)
        self.m.receive_emergency(6, 6)
        self.assertTrue(self.m.needs_interrupt())
        self.m.finish(searching, 'canceled')
        self._go(self.m.next_target())
        self.assertEqual(self.m.next_target(), searching)

    def test_search_unreachable_waypoint_skipped(self):
        self.m.configure_search([(3, 4), (5, 6)])
        self.m.receive_normal_goal(1, 1, 1)
        self.m.receive_normal_goal(2, 2, 2)
        self._go(self.m.next_target())
        self._go(self.m.next_target())
        target = self.m.next_target()
        self.m.start(target)
        self.m.finish(target, 'failed')
        self.assertEqual(self.m.next_target().x, 5)

    def test_no_goals(self):
        self.assertIsNone(self.m.next_target())
        self.assertEqual(self.m.state, 'WAITING_FOR_GOAL1')

    def test_goal2_alone(self):
        self.m.receive_normal_goal(2, 4, 5)
        self.assertIsNone(self.m.next_target())

    def test_goal1_then_wait_goal2(self):
        self.m.receive_normal_goal(1, 1, 2)
        self._go(self.m.next_target())
        self.assertEqual(self.m.state, 'WAITING_FOR_GOAL2')
        self.assertIsNone(self.m.next_target())

    def test_both_goals_in_order(self):
        self.m.receive_normal_goal(1, 1, 2)
        self.m.receive_normal_goal(2, 3, 4)
        first = self.m.next_target()
        self.assertEqual(first.kind, 'goal1')
        self._go(first)
        second = self.m.next_target()
        self.assertEqual(second.kind, 'goal2')
        self._go(second)
        self.assertEqual(self.m.state, 'GOALS_REACHED_AWAITING_HUMAN')

    def test_goal2_first_then_goal1(self):
        self.m.receive_normal_goal(2, 4, 6)
        self.m.receive_normal_goal(1, 1, 1)
        self.assertEqual(self.m.next_target().kind, 'goal1')

    def test_emergency_without_normal_goals(self):
        self.m.receive_emergency(-1, 2)
        self.assertEqual(self.m.next_target().kind, 'goal3')
        self._go(self.m.next_target())
        self.assertIsNone(self.m.next_target())

    def test_emergency_preempts_goal1_then_resumes_goal1_and_goal2(self):
        self.m.receive_normal_goal(1, 1, 2)
        self.m.receive_normal_goal(2, 3, 4)
        first = self.m.next_target()
        self.m.start(first)
        self.m.receive_emergency(9, 8)
        self.assertTrue(self.m.needs_interrupt())
        self.m.finish(first, 'canceled')
        emergency = self.m.next_target()
        self.assertEqual(emergency.kind, 'goal3')
        self._go(emergency)
        self.assertEqual(self.m.next_target().kind, 'goal1')
        self._go(self.m.next_target())
        self.assertEqual(self.m.next_target().kind, 'goal2')

    def test_emergency_preempts_goal2_and_resumes(self):
        self.m.receive_normal_goal(1, 1, 2)
        self.m.receive_normal_goal(2, 3, 4)
        self._go(self.m.next_target())
        second = self.m.next_target()
        self.m.start(second)
        self.m.receive_emergency(9, 8)
        self.m.finish(second, 'canceled')
        self._go(self.m.next_target())
        self.assertEqual(self.m.next_target().kind, 'goal2')

    def test_emergency_during_goal1_not_marked_reached_when_canceled(self):
        self.m.receive_normal_goal(1, 1, 2)
        first = self.m.next_target()
        self.m.start(first)
        self.m.receive_emergency(8, 8)
        self.m.finish(first, 'canceled')
        self.assertFalse(self.m.goal1_reached)

    def test_goal3_already_arrived_takes_priority(self):
        self.m.receive_emergency(8, 8)
        self.m.receive_normal_goal(1, 1, 2)
        self.assertEqual(self.m.next_target().kind, 'goal3')

    def test_new_emergency_replaces_current_emergency(self):
        self.m.receive_emergency(3, 3)
        prior = self.m.next_target()
        self.m.start(prior)
        self.m.receive_emergency(5, 5)
        self.assertTrue(self.m.needs_interrupt())
        self.m.finish(prior, 'canceled')
        self.assertEqual(self.m.next_target().x, 5)

    def test_repeated_emergency_is_ignored_when_pending(self):
        self.assertTrue(self.m.receive_emergency(3, 3))
        self.assertFalse(self.m.receive_emergency(3, 3))

    def test_new_emergency_after_previous_reached(self):
        self.m.receive_emergency(3, 3)
        self._go(self.m.next_target())
        self.assertTrue(self.m.receive_emergency(3, 3))
        self.assertEqual(self.m.next_target().kind, 'goal3')

    def test_human_found_is_terminal_and_higher_priority(self):
        self.m.receive_normal_goal(1, 1, 2)
        first = self.m.next_target()
        self.m.start(first)
        self.m.receive_emergency(9, 8)
        self.assertTrue(self.m.report_human(3.5, 7.2))
        self.assertTrue(self.m.needs_interrupt())
        self.m.finish(first, 'canceled')
        self.assertIsNone(self.m.next_target())
        self.assertEqual(self.m.state, 'HUMAN_FOUND')

    def test_no_repeated_human_detection(self):
        self.assertTrue(self.m.report_human(1, 2))
        self.assertFalse(self.m.report_human(3, 4))

    def test_normal_failure_blocks_base_but_not_emergency(self):
        self.m.receive_normal_goal(1, 1, 1)
        first = self.m.next_target()
        self.m.start(first)
        self.m.finish(first, 'failed')
        self.assertIsNone(self.m.next_target())
        self.m.receive_emergency(2, 2)
        self.assertEqual(self.m.next_target().kind, 'goal3')

    def test_failed_emergency_does_not_mark_reached(self):
        self.m.receive_normal_goal(1, 1, 1)
        self.m.receive_emergency(2, 2)
        em = self.m.next_target()
        self.m.start(em)
        self.m.finish(em, 'failed')
        self.assertEqual(self.m.next_target().kind, 'goal1')

    def test_first_normal_goal_message_wins(self):
        self.assertTrue(self.m.receive_normal_goal(1, 1, 2))
        self.assertFalse(self.m.receive_normal_goal(1, 9, 9))
        self.assertEqual(self.m.goal1.x, 1)

    def test_out_of_order_start_disallowed(self):
        self.m.receive_normal_goal(1, 1, 2)
        self.m.receive_normal_goal(2, 3, 4)
        with self.assertRaises(ValueError):
            self.m.start(self.m.goal2)

    def test_bad_coordinates_disallowed(self):
        with self.assertRaises(ValueError):
            self.m.receive_emergency(float('nan'), 3)
        with self.assertRaises(ValueError):
            self.m.report_human(3, float('inf'))


if __name__ == '__main__':
    unittest.main()
