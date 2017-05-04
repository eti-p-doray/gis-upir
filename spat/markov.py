import math

#from fibonacci_heap_mod import Fibonacci_heap
from spat.priority_queue import PriorityQueue


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
    def __init__(self, transition_generator, state_projection, transition_cost_fcn):
        self.transition_generator = transition_generator
        self.state_projection = state_projection
        self.transition_cost_fcn = transition_cost_fcn

    def find_best(self, origin, goal, progress_fcn, heuristic_fcn):

        states = {}

        def at(key):
            if key not in states:
                state, state_cost = self.state_projection(key)
                states[key] = (state, state_cost)
            else:
                state, state_cost = states[key]
            return state, state_cost

        queue = PriorityQueue()

        state, cost = at(origin)
        heuristic = heuristic_fcn(origin)

        queue.put(origin, cost + heuristic)
        cost_table = {
            origin: cost
        }
        backtrack_chain = {
            origin: None
        }
        visited = {}
        progress_table = {}

        current_key = None
        while not queue.empty():
            current_key = queue.get()
            if current_key == goal:
                break

            if current_key in progress_table:
                continue

            i, j = progress_fcn(current_key)

            if i in progress_table and j < progress_table[i]:
                continue

            print("Visit: ", current_key)

            current_state, _ = at(current_key)
            base_cost = cost_table[current_key]

            visited[current_key] = True
            progress_table[j] = i

            for next_key in self.transition_generator(current_state):
                if next_key in visited:
                    continue
                i, j = progress_fcn(current_key)
                if i in progress_table and j < progress_table[i]:
                    continue

                print("Discover: ", next_key)

                next_state, state_cost = at(next_key)

                if state_cost == math.inf:
                    continue

                transition_cost = self.transition_cost_fcn(current_state, next_state)
                new_cost = base_cost + state_cost + transition_cost

                if new_cost == math.inf or next_key in cost_table and new_cost >= cost_table[next_key]:
                    continue

                heuristic = heuristic_fcn(next_state)

                priority = new_cost + heuristic

                queue.put(next_key, priority)
                cost_table[next_key] = new_cost
                backtrack_chain[next_key] = current_key

        if current_key != goal:
            return None

        def backtrack():
            node = current_key
            while node is not None:
                yield node
                node = backtrack_chain[node]

        return reversed(list(backtrack())), states
