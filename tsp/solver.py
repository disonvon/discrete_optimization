#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
from collections import namedtuple
from docplex.mp.model import Model
from tsp_solver import *

Point = namedtuple("Point", ['x', 'y'])

def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

def mip_tsp(points, time_threshold=None):
    nodeCount = len(points)
    #set up the model
    m = Model(name='tsp')
    #add xij variable
    m.x = m.binary_var_matrix(nodeCount,nodeCount,name='x')
    #add variable mu_i
    m.mu = m.continuous_var_list(nodeCount,lb=0,name='mu')

    #add sum_xij=1 for all j
    for i in range(nodeCount):
        m.add_constraint(m.sum(m.x[i,j] for j in range(nodeCount) if i != j) == 1, 'leave')

     #add sum_xij=1 for all i
    for j in range(nodeCount):
        m.add_constraint(m.sum(m.x[i,j] for i in range(nodeCount) if i != j) == 1,'enter')
    
    #add constraints to eliminate subtour
    for i in range(1, nodeCount):
        for j in range(1, nodeCount):
            if i == j:
                continue
            m.add_constraint(m.mu[i] - m.mu[j] + nodeCount*m.x[i,j] <= nodeCount - 1, 'subtour')
    
    #add obj function: sum_xij*c_ij
    obj = m.sum(length(points[i], points[j])*m.x[i,j] for i in range(nodeCount) for j in range(nodeCount))    
    #set time limit(seconds) for each soltuion
    m.set_time_limit(time_threshold)
    m.minimize(obj)
    m.solve()
    # print('--------------status---------------------------------- \n', m.get_solve_status())

    if 'optimal' in m.solve_details.status:
        status = 1
    else:
        status = 0
    # m.export_as_lp('/Users/zhiweifeng/Documents/git/Engines/discrete_optimization/tsp')
    sol = []
    i = 0
    sol.append(i)
    while len(sol) < nodeCount:
        for j in range(nodeCount):
            if i != j and int(m.x[i,j].solution_value) == 1:
                sol.append(j)
                i = j
                break
    return m.objective_value, status, sol

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    nodeCount = int(lines[0])

    points = []
    for i in range(1, nodeCount+1):
        line = lines[i]
        parts = line.split()
        points.append(Point(float(parts[0]), float(parts[1])))


    opt = 0
    solution = []
    if len(points) <= 200:
        # solve it with mix integer programming

        obj, opt, solution = mip_tsp(points, time_threshold=1200)
    
    if opt == 0:
        
        solver = tsp_solver(solution=solution, points=points)

        if len(points) < 1500:
            #solve it with 2-opt
            #time limit for the algorithm to stop
            obj, opt, solution = solver.two_opt_solver(time_threshold=1200)
        else:
             obj, opt, solution = solver.greedy()
           
    
    # prepare the solution in the specified output format
    output_data = '%.2f' % obj + ' ' + str(opt) + '\n'
    output_data += ' '.join(map(str, solution))
    
    return output_data


import sys

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/tsp_51_1)')

# file_location = './data/tsp_5_1'.strip()
# with open(file_location, 'r') as input_data_file:
#     input_data = input_data_file.read()
#     print(solve_it(input_data))