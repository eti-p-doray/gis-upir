import math
import logging

#from fibonacci_heap_mod import Fibonacci_heap
from spat.priority_queue import PriorityQueue


def no_handicap(key):
    return 0.0

def identity_projection(key):
    return key


class MarkovGraph:
    """ Represents a markov chain as a graph of virtual nodes.

    A markov chain is a stochastic process in which the conditional probability
    distribution of future states of the process depends only upon
    the present state.

    States in the graph are represented as hashable keys. The function |state_projection| maps
    those keys to the state values.

    Transitions in the chain are discovered through |transition_generator|, a
    generator of adjacent hashable state keys, that takes the current state value as argument.

    """
    def __init__(self, transition_generator, state_cost_fcn, transition_cost_fcn,
                 state_projection=identity_projection,
                 handicap_fcn = no_handicap):
        self.transition_generator = transition_generator
        self.state_projection = state_projection
        self.state_cost_fcn = state_cost_fcn
        self.transition_cost_fcn = transition_cost_fcn
        self.handicap_fcn = handicap_fcn

    def find_best(self, origin, goal, heuristic_fcn,
                  priority_threshold=math.inf, progress_fcn=None,
                  max_visited=None):

        queue = PriorityQueue()

        origin_node = self.state_projection(origin)

        cost = self.state_cost_fcn(origin_node)
        handicap_cost = self.handicap_fcn(origin_node)
        heuristic = heuristic_fcn(origin_node)

        queue.put(origin, cost + handicap_cost + heuristic)
        cost_table = {
            origin: (cost, cost + handicap_cost)
        }
        priority_table = {
            origin: cost + handicap_cost + heuristic
        }
        backtrack_chain = {
            origin: None
        }
        visited = {}
        progress_table = {}

        current_key = None

        logging.debug("Max Visited: %d", max_visited)
        while not queue.empty():
            current_key = queue.get()
            if current_key == goal:
                break

            if current_key in visited:
                continue

            if progress_fcn:
                step, progress = progress_fcn(current_key)
                if step in progress_table and progress < progress_table[step]:
                    continue
                progress_table[step] = progress
            visited[current_key] = True
            if max_visited is not None and len(visited) > max_visited:
                break

            current_node = self.state_projection(current_key)

            logging.debug("Visited: %d", len(visited))
            logging.debug("Visit: %s", str(current_node))

            base_cost = cost_table[current_key][0]

            for next_key in self.transition_generator(current_node):
                if next_key in visited:
                    continue

                if progress_fcn:
                    step, progress = progress_fcn(next_key)
                    if step in progress_table and progress < progress_table[step]:
                        continue

                next_node = self.state_projection(next_key)

                logging.debug("Discover: %s", str(next_node))

                state_cost = self.state_cost_fcn(next_node)

                if state_cost == math.inf:
                    continue

                transition_cost = self.transition_cost_fcn(current_node, next_node)
                handicap_cost = self.handicap_fcn(next_node)
                new_cost = base_cost + state_cost + transition_cost

                if new_cost == math.inf:
                    continue
                #    (next_node in cost_table and new_cost + handicap_cost >= cost_table[next_key][1])):
                #    continue

                heuristic = heuristic_fcn(next_node)
                priority = new_cost + handicap_cost + heuristic
                if priority >= priority_threshold:
                    continue
                if next_key in priority_table and priority >= priority_table[next_key]:
                    continue

                logging.debug("priority, cost, heuristic: %.4f, %.4f, %.4f", priority, new_cost + handicap_cost, heuristic)

                queue.put(next_key, priority)
                cost_table[next_key] = (new_cost, new_cost + handicap_cost)
                priority_table[next_key] = priority
                backtrack_chain[next_key] = current_key

        if current_key != goal:
            return None

        logging.info("priority, cost: %.4f, %.4f", priority_table[goal], cost_table[goal][0])

        def backtrack():
            key = current_key
            while key is not None:
                yield key
                key = backtrack_chain[key]

        return reversed(list(backtrack())), priority_table[goal]
