#!/usr/bin/python
# -*- coding: utf-8 -*-

from collections import namedtuple
import math
from docplex.mp.model import Model


Point = namedtuple("Point", ['x', 'y'])
Facility = namedtuple("Facility", ['index', 'setup_cost', 'capacity', 'location'])
Customer = namedtuple("Customer", ['index', 'demand', 'location'])

def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

def facility_solver(facility_count, customer_count, facilities, customers, time_threshold=300):
    m = Model('facility')
    #add binary variables xij, customer i to facility j
    m.x = m.binary_var_matrix(customer_count, facility_count, name='x')

    #add binary variable for fixed cost part yj
    m.y = m.binary_var_list(facility_count, name='y')

    #add capacity constraints
    for j in range(facility_count):
        m.add_constraint(m.sum(m.x[i,j] * customers[i].demand for i in range(customer_count)) <= facilities[j].capacity, 'capacity_'+str(j))

    #add each customer served only by 1 facility
    for i in range(customer_count):
        m.add_constraint(m.sum(m.x[i,j] for j in range(facility_count)) == 1, 'customer_'+str(i))

    #add yj constraints
    for j in range(facility_count):
        m.add_constraint(m.sum(m.x[i,j] for i in range(customer_count)) <= customer_count*m.y[j], 'upperbounder_y_'+str(j))
    #add obj function: sum_sj*yj + sum_distance
    obj = m.sum(length(customers[i].location, facilities[j].location)*m.x[i,j] for i in range(customer_count) \
        for j in range(facility_count)) + m.sum(facilities[j].setup_cost*m.y[j] for j in range(facility_count))    
    #set time limit(seconds) for each soltuion
    m.set_time_limit(time_threshold)
    m.minimize(obj)
    m.solve()
    if 'optimal' in m.solve_details.status:
        status = 1
    else:
        status = 0

    solution = []
    for i in range(customer_count):
        for j in range(facility_count):
            if int(m.x[i,j].solution_value) == 1:
                solution.append(j)
                continue

    return m.objective_value, status, solution


def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    parts = lines[0].split()
    facility_count = int(parts[0])
    customer_count = int(parts[1])
    
    facilities = []
    for i in range(1, facility_count+1):
        parts = lines[i].split()
        facilities.append(Facility(i-1, float(parts[0]), int(parts[1]), Point(float(parts[2]), float(parts[3])) ))

    customers = []
    for i in range(facility_count+1, facility_count+1+customer_count):
        parts = lines[i].split()
        customers.append(Customer(i-1-facility_count, int(parts[0]), Point(float(parts[1]), float(parts[2]))))

    #solve it with mip
    obj, opt, solution = facility_solver(facility_count, customer_count, facilities, customers, time_threshold=1800)
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
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/fl_16_2)')

