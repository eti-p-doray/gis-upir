

class MatchedSegment:

    class Bound:
        def __init__(self, projection, attached: bool, idx):
            self.projection = projection
            self.attached = attached
            self.idx = idx

    def __init__(self, edge, geometry, begin: Bound, end: Bound):
        self.edge = edge
        self.geometry = geometry
        self.begin = begin
        self.end = end
