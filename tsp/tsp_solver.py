import math
from itertools import combinations
from time import time


class tsp_solver():

    def __init__(self, solution, points):
        self.threshold = 10**-4
        self.points = points
        self.opt = 0
        if solution:
            self.cycle = solution + [0]
        else:
            self.cycle = list(range(len(points))) + [0]
        self.obj_val = self.cycle_length()

    @staticmethod
    def point_dist(p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def is_valid_sln(self):
        return len(set(self.cycle[:-1])) == len(self.points) == len(self.cycle[:-1])

    def edge_dist(self, v1, v2):
        p1 = self.points[v1]
        p2 = self.points[v2]
        return self.point_dist(p1,p2)

    def cycle_length(self):
        return sum(self.edge_dist(v1, v2) for v1, v2 in zip(self.cycle[:-1], self.cycle[1:]))

    def greedy(self):
        cycle = [0]
        candidates = set(self.cycle[1:-1])
        while candidates:
            curr_point = cycle[-1]
            nearest_neighbor = None
            nearest_dist = math.inf
            for neighbor in candidates:
                neighbor_dist = self.edge_dist(curr_point, neighbor)
                if neighbor_dist < nearest_dist:
                    nearest_neighbor = neighbor
                    nearest_dist = neighbor_dist
            cycle.append(nearest_neighbor)
            candidates.remove(nearest_neighbor)
        cycle.append(0)
        self.cycle = cycle
        self.obj_val = self.cycle_length()
        return self.obj_val, self.opt, self.cycle

    def swap(self, start, end):
        improved = False
        new_cycle = self.cycle[:start] + self.cycle[start:end+1][::-1] + self.cycle[end+1:]
        new_obj_val = self.obj_val - (self.edge_dist(self.cycle[start-1], self.cycle[start]) + 
                                        self.edge_dist(self.cycle[end], self.cycle[end+1])) + \
                                        (self.edge_dist(new_cycle[start-1], new_cycle[start]) + 
                                        self.edge_dist(new_cycle[end], new_cycle[end+1]))
        if new_obj_val < self.obj_val - self.threshold:
            self.cycle = new_cycle
            self.obj_val = new_obj_val
            improved = True
            return improved

    def two_opt_solver(self, time_threshold=None):
        improved = True
        t = time()
        while improved:
            if time_threshold and time() - t >= time_threshold:
                break
            improved = False
            for start, end in combinations(range(1, len(self.cycle) - 1), 2):
                if self.swap(start, end):
                    improved = True
                    break
        return self.obj_val, self.opt, self.cycle[:-1]

