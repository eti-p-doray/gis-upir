import math

from fibonacci_heap_mod import Fibonacci_heap


class MarkovChain:
    """ Represents a markov chain as a graph of virtual nodes.
  
    A markov chain is a stochastic process in which the conditional probability 
    distribution of future states of the process depends only upon 
    the present state.
    
    States in the graph are represented as hashable keys. The function |state_projection| maps
    those keys to the state values.
  
    Transitions in the chain are discovered through |transition_generator|, a 
    generator of adjacent hashable state keys, that takes the current state key as argument.
  
    """
    def __init__(self, transition_generator, state_projection, transition_cost_fcn):
        self.transition_generator = transition_generator
        self.state_projection = state_projection
        self.transition_cost_fcn = transition_cost_fcn

        self.states = {}

    def at(self, key):
        if key not in self.states:
            state, state_cost = self.state_projection(key)
            self.states[key] = (state, state_cost)
        else:
            state, state_cost = self.states[key]
        return state, state_cost

    def find_best(self, origin, goal, heuristic_fcn):
        queue = Fibonacci_heap()

        state, cost = self.at(origin)
        heuristic = heuristic_fcn(origin)

        queue.enqueue(origin, cost + heuristic)
        heuristic_table = {
            origin: heuristic
        }
        cost_table = {
            origin: cost
        }
        backtrack_chain = {
            origin: None
        }
        visited = {}

        while queue:
            current_key = queue.dequeue_min().get_value()
            current_state, _ = self.at(current_key)
            base_cost = cost_table[current_key]

            if current_key == goal:
                break

            if current_key in visited:
                continue
            visited[current_key] = True

            for next_key in self.transition_generator(current_key):

                next_state, state_cost = self.at(next_key)

                if state_cost == math.inf:
                    continue

                transition_cost = self.transition_cost_fcn(current_state, next_state)
                new_cost = base_cost + state_cost + transition_cost

                if next_key in cost_table and new_cost >= cost_table[next_key]:
                    continue

                if next_key in heuristic_table:
                    heuristic = heuristic_table[next_key]
                else:
                    heuristic = heuristic_fcn(next_state)
                    heuristic_table[next_key] = heuristic

                priority = new_cost + heuristic
                if next_key in cost_table:
                    queue.decrease_key(next_key, priority)
                else:
                    queue.enqueue(next_key, priority)
                cost_table[next_key] = new_cost
                backtrack_chain[next_key] = current_key

        if current_key != goal:
            return None

        def backtrack():
            node = current_key
            while node is not None:
                yield node
                node = backtrack_chain[node]

        return reversed(list(backtrack()))
