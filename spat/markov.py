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
    def __init__(self, transition_generator, state_projection, state_cost_fcn, transition_cost_fcn, handicap_fcn):
        self.transition_generator = transition_generator
        self.state_projection = state_projection
        self.state_cost_fcn = state_cost_fcn
        self.transition_cost_fcn = transition_cost_fcn
        self.handicap_fcn = handicap_fcn

    def find_best(self, origin, goal, progress_fcn, heuristic_fcn):

        queue = PriorityQueue()

        origin_node = self.state_projection(origin)

        cost = self.state_cost_fcn(origin_node)
        handicap_cost = self.handicap_fcn(origin_node)
        heuristic = heuristic_fcn(origin_node)

        queue.put((origin_node, origin), cost + heuristic)
        cost_table = {
            origin: (cost, cost + handicap_cost)
        }
        backtrack_chain = {
            origin: (None, None)
        }
        visited = {}
        progress_table = {}

        current_key = None
        while not queue.empty():
            current_node, current_key = queue.get()
            if current_key == goal:
                break

            if current_key in visited:
                continue

            step, progress = progress_fcn(current_key)
            if step in progress_table and progress <= progress_table[step]:
                continue

            print("Visit: ", current_node)

            base_cost = cost_table[current_key][0]

            visited[current_key] = True
            progress_table[step] = progress

            for next_key in self.transition_generator(current_node):
                if next_key in visited:
                    continue

                step, progress = progress_fcn(next_key)
                if step in progress_table and progress <= progress_table[step]:
                    continue

                next_node = self.state_projection(next_key)

                print("Discover: ", next_node)

                state_cost = self.state_cost_fcn(next_node)

                if state_cost == math.inf:
                    continue

                transition_cost = self.transition_cost_fcn(current_node, next_node)
                handicap_cost = self.handicap_fcn(next_node)
                new_cost = base_cost + state_cost + transition_cost

                if (new_cost == math.inf or
                    (next_node in cost_table and new_cost + handicap_cost >= cost_table[next_key][1])):
                    continue

                heuristic = heuristic_fcn(next_node)
                priority = new_cost + handicap_cost + heuristic

                print(priority)

                queue.put((next_node, next_key), priority)
                cost_table[next_key] = (new_cost, new_cost + handicap_cost)
                backtrack_chain[next_key] = (current_key, current_node)

        if current_key != goal:
            return None

        def backtrack():
            key = current_key
            node = current_node
            while key is not None:
                yield key, node
                key, node = backtrack_chain[key]

        return reversed(list(backtrack()))
