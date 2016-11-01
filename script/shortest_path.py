import heapq
import pickle

class PriorityQueue:
  def __init__(self):
    self.elements = []
  
  def empty(self):
    return len(self.elements) == 0
  
  def put(self, item, priority):
    heapq.heappush(self.elements, (priority, item))
  
  def get(self):
    return heapq.heappop(self.elements)[1]

def a_star_search(g, start, goal, cost, heuristic):
  frontier = PriorityQueue()
  frontier.put(start, 0)
  came_from = {}
  cost_so_far = {}
  came_from[start] = None
  cost_so_far[start] = 0
  
  while not frontier.empty():
    current = frontier.get()
      
    if current == goal:
      break
      
    for next in g.neighbors(current):
      new_cost = cost_so_far[current] + cost(g, current, next)
      if next not in cost_so_far or new_cost < cost_so_far[next]:
        cost_so_far[next] = new_cost
        priority = new_cost + heuristic(g, next, goal)
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