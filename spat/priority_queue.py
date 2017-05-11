import heapq


class PriorityQueue:
    def __init__(self, max_size = None):
        self.elements = []
        self.max_size = max_size

    def empty(self):
        return len(self.elements) == 0

    def size(self):
        return len(self.elements)

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
        if self.max_size is not None and len(self.elements) > self.max_size:
            self.elements = self.elements[0:self.max_size]

    def get(self):
        return heapq.heappop(self.elements)[1]
