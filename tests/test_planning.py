import math
import unittest
from minibot.planning import Grid, Greeting


class PlannerTests(unittest.TestCase):
    def test_obstacle_inflation_keeps_path_clear(self):
        data = [0]*1600
        for row in range(8, 32):
            data[row*40+20] = 100
        grid = Grid(40, 40, .1, (0, 0, 0), data)
        path = grid.route(grid.cell(1, 2), grid.cell(3, 2))
        self.assertTrue(path)
        self.assertTrue(any(y < .8 or y > 3.2 for x, y in path))
        self.assertTrue(all(abs(x-2.05) >= .15 for x, y in path if .8 <= y <= 3.2))

    def test_unknown_barrier_not_crossed(self):
        data = [0]*400
        for row in range(20):
            data[row*20+10] = -1
        grid = Grid(20, 20, .1, (0, 0, 0), data)
        self.assertFalse(grid.route(grid.cell(.5, 1), grid.cell(1.5, 1)))

    def test_rotated_grid_roundtrip(self):
        grid = Grid(20, 20, .1, (4, 2, math.pi/2), [0]*400)
        i = 210
        self.assertEqual(i, grid.cell(*grid.point(i)))
        self.assertIsNone(grid.cell(30, 30))

    def test_frontier_is_reachable_and_clear(self):
        data = [0 if i % 40 < 30 else -1 for i in range(1600)]
        grid = Grid(40, 40, .1, (0, 0, 0), data)
        start = grid.cell(1, 2)
        target = grid.explore(start, [])
        self.assertIn(target, grid.free)
        self.assertGreater(grid.point(target)[0], 2.5)
        self.assertTrue(grid.route(start, target))

    def test_greeting_cooldown_and_absence(self):
        greeting = Greeting()
        self.assertTrue(greeting.start(0))
        self.assertEqual(greeting.until, 10)
        self.assertFalse(greeting.start(11))
        greeting.observe(True, 65)
        self.assertFalse(greeting.start(65))
        greeting.observe(False, 66)
        greeting.observe(False, 69)
        self.assertTrue(greeting.start(69))


if __name__ == '__main__':
    unittest.main()
