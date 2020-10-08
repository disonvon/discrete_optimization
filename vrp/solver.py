#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
from collections import namedtuple
from docplex.mp.model import Model


Customer = namedtuple("Customer", ['index', 'demand', 'x', 'y'])

def length(customer1, customer2):
    return math.sqrt((customer1.x - customer2.x)**2 + (customer1.y - customer2.y)**2)

def vrp_mip_solver(customers, customer_count, vehicle_count, vehicle_capacity, time_threshold=None):
    #set up the model
    m = Model(name='vrp')
    #add xijk variable, node i to node j of vehicle k
    m.x = m.binary_var_cube(customer_count, customer_count, vehicle_count, name='x')
    # #add variable mu_ik
    m.mu = m.continuous_var_matrix(customer_count, vehicle_count, name='mu')

    #add sum_xijk=1 for all i
    for i in range(1, customer_count):
        m.add_constraint(m.sum(m.x[i,j,k] for j in range(customer_count) for k in range(vehicle_count) \
            if i != j) == 1, 'leave_node_i_'+str(i))

     #add sum_xijk=1 for all j
    for j in range(1, customer_count):
        m.add_constraint(m.sum(m.x[i,j,k] for i in range(customer_count) for k in range(vehicle_count) \
            if i != j) == 1,'enter_node_j_'+str(j))
    
    # #add constraints to eliminate subtour

    for i in range(1, customer_count):
        for j in range(1, customer_count):
            for k in range(vehicle_count):
                if i == j:
                    continue
                m.add_constraint(m.mu[i,k] - m.mu[j,k] + customer_count*m.x[i,j,k] <= customer_count - 1, 'subtour_'+str(i)+'_'+str(j)+'_'+str(k))
    
    #add constraints start from depot
    for k in range(vehicle_count):
        m.add_constraint(m.sum(m.x[0,j,k] for j in range(1, customer_count)) == 1,'start_depot_vehicle_'+str(k))
    
    #add constraints end at depot
    for k in range(vehicle_count):
        m.add_constraint(m.sum(m.x[i,0,k] for i in range(1, customer_count)) == 1,'end_depot_vehicle_'+str(k))

    #add constraints to keep flow
    for k in range(vehicle_count):
        for h in range(0, customer_count):
            m.add_constraint(m.sum(m.x[i,h,k] for i in range(customer_count) if i != h) - \
                m.sum(m.x[h,j,k] for j in range(customer_count) if j != h) == 0, 'vehicle_'+str(k)+'_flow_node_'+str(h))
             

    #capacity constraints
    for k in range(vehicle_count):
        m.add_constraint(m.sum(customers[i].demand*m.x[i,j,k] for i in range(customer_count) \
            for j in range(customer_count) if i != j) <= vehicle_capacity,'capacity_vehicle_'+str(k))
            
    #add obj function: sum_xij*c_ij
    obj = m.sum(length(customers[i], customers[j])*m.x[i,j,k] for i in range(customer_count) \
        for j in range(customer_count) for k in range(vehicle_count))    
    #set time limit(seconds) for each soltuion
    m.set_time_limit(time_threshold)
    m.minimize(obj)
    m.solve()
    # print('--------------status---------------------------------- \n', m.get_solve_status())
    # m.export_as_lp('/Users/zhiweifeng/Documents/git/Engines/discrete_optimization/vrp')
    # for k in range(vehicle_count):
    #     for j in range(customer_count):
    #         for i in range(customer_count):
    #             if int(m.x[i,j,k].solution_value) == 1:
    #                 print('x_'+str(i)+'_'+str(j)+'_'+str(k), int(m.x[i,j,k].solution_value))
    if 'optimal' in m.solve_details.status:
        status = 1
    else:
        status = 0
    vehicle_tours = []
    
    for k in range(vehicle_count):
        # print "Start Vehicle: ",v
        vehicle_tours.append([])
        vehicle_tours[k].append(0)
        i = 0

        while len(vehicle_tours[k]) < customer_count + 1:
            if int(m.x[0,0,k].solution_value) == 1:
                vehicle_tours[k].append(0)
                break
            done = False
            for j in range(0,customer_count):
                if int(m.x[i,j,k].solution_value) == 1:
                    vehicle_tours[k].append(j)
                    i = j
                    if j == 0:
                        done = True
                        break
            # if len(vehicle_tours[k]) >= 2 and vehicle_tours[k][-1] == 0:
            if done:
                break
    return m.objective_value, status, vehicle_tours


def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    parts = lines[0].split()
    customer_count = int(parts[0])
    vehicle_count = int(parts[1])
    vehicle_capacity = int(parts[2])
    
    customers = []
    for i in range(1, customer_count+1):
        line = lines[i]
        parts = line.split()
        customers.append(Customer(i-1, int(parts[0]), float(parts[1]), float(parts[2])))

    # #the depot is always the first customer in the input
    depot = customers[0] 


    # # build a trivial solution
    # # assign customers to vehicles starting by the largest customer demands
    # vehicle_tours = []
    
    # remaining_customers = set(customers)
    # remaining_customers.remove(depot)
    
    # for v in range(0, vehicle_count):
    #     # print "Start Vehicle: ",v
    #     vehicle_tours.append([])
    #     capacity_remaining = vehicle_capacity
    #     while sum([capacity_remaining >= customer.demand for customer in remaining_customers]) > 0:
    #         used = set()
    #         order = sorted(remaining_customers, key=lambda customer: -customer.demand*customer_count + customer.index)
    #         for customer in order:
    #             if capacity_remaining >= customer.demand:
    #                 capacity_remaining -= customer.demand
    #                 vehicle_tours[v].append(customer)
    #                 # print '   add', ci, capacity_remaining
    #                 used.add(customer)
    #         remaining_customers -= used

    # # checks that the number of customers served is correct
    # assert sum([len(v) for v in vehicle_tours]) == len(customers) - 1

    # # calculate the cost of the solution; for each vehicle the length of the route
    # obj = 0
    # for v in range(0, vehicle_count):
    #     vehicle_tour = vehicle_tours[v]
    #     if len(vehicle_tour) > 0:
    #         obj += length(depot,vehicle_tour[0])
    #         for i in range(0, len(vehicle_tour)-1):
    #             obj += length(vehicle_tour[i],vehicle_tour[i+1])
    #         obj += length(vehicle_tour[-1],depot)

    # solve it with mip model cplex
    obj, opt, vehicle_tours = vrp_mip_solver(customers, customer_count, vehicle_count, vehicle_capacity, time_threshold=900)
    # prepare the solution in the specified output format
    outputData = '%.2f' % obj + ' ' + str(opt) + '\n'
    for v in range(0, vehicle_count):
        # outputData += str(depot.index) + ' ' + ' '.join([str(customer.index) for customer in vehicle_tours[v]]) + ' ' + str(depot.index) + '\n'
        outputData += ' '.join([str(customer) for customer in vehicle_tours[v]]) + ' ' + '\n'


    return outputData


import sys

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:

        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/vrp_5_4_1)')

