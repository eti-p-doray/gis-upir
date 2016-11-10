from priority_queue import *

def a_star_search(g, starts, goals, cost, heuristic):
  frontier = PriorityQueue()
  came_from = {}
  cost_so_far = {}
  remaining_goal = 0
  for s in starts:
    frontier.put(s[0], s[1])
    came_from[s[0]] = None
    cost_so_far[s[0]] = s[1]
    #state_so_far = s[2]
  
  while not frontier.empty():
    current = frontier.get()
    
    if current in goals:
      remaining_goal -= 1
      if remaining_goal == 0:
        break
      
    for next in g.neighbors(current):
      edge_cost = cost(g, current, next)
      new_cost = cost_so_far[current] + edge_cost
      if next not in cost_so_far or new_cost < cost_so_far[next]:
        cost_so_far[next] = new_cost
        #state_so_far[next] = state
        priority = new_cost + heuristic(g, next)
        frontier.put(next, priority)
        came_from[next] = current
  
  return came_from, cost_so_far

def euclidian_distance(g, i, j):
  return g.node[i]['geometry'].distance(g.node[j]['geometry'])

def travel_cost(g, i, j):
  return g.way(i, j).length


#with open('data/facilities/montreal.pickle', 'r') as f:
#  n = pickle.load(f)
#
#print a_star_search(n.graph, 0, 10, travel_cost, euclidian_distance)