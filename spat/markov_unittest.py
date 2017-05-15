import unittest

from spat import markov


class TestMarkov(unittest.TestCase):

    def test_search_best_simple(self):
        def adjacent(key):
            yield key + 1

        def state(current_key):
            return current_key, 0

        def cost(previous_state, current_state):
            return 1

        def heuristic(state):
            return 4 - state

        chain = markov.MarkovGraph(adjacent, state, cost)
        path = chain.find_best(0, 4, heuristic)
        self.assertEqual(list(path), [0, 1, 2, 3, 4])


if __name__ == '__main__':
    unittest.main()