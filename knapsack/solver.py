#!/usr/bin/python
# -*- coding: utf-8 -*-

from collections import namedtuple
from docplex.mp.model import Model
import numpy as np

Item = namedtuple("Item", ['index', 'value', 'weight'])


def knapsack_mip(num_vars, items, capacity):
    # create one model instance, with a name
    m = Model(name='knapsack')
    #set up variables
    m.taken = m.binary_var_dict(items)
    #set up obj function
    obj = m.sum(m.taken[item]*item.value for item in items)
    #set up constraints
    m.add_constraint(m.sum(m.taken[item]*item.weight for item in items) <= capacity)
    m.maximize(obj)
    m.solve()
    if m.get_solve_status() != None:
        status = 1
    else:
        status = 0
    return m.objective_value, status, [int(m.taken[item].solution_value) for item in items]

def dynamic_model(num_vars, items, capacity):
    taken = [0]*num_vars
    values = np.zeros((num_vars+1, capacity+1))

    for i in range(num_vars+1):
        if i > 0:
            value = items[i-1].value
            weight = items[i-1].weight
        for j in range(capacity+1):
            if (i == 0 or j == 0):
                continue
            elif weight > j:
                values[i][j] = values[i-1][j]
            else:
                values[i][j] = max(values[i-1][j-weight] + value, values[i-1,j])
    
    for i in reversed(range(num_vars)):
        total_weight = capacity
        if values[i][total_weight] == values[i+1][total_weight]:
            continue
        else:
            taken[i] = 1
            total_weight -= items[i].weight
    return values[-1][-1], 1, taken




def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    firstLine = lines[0].split()
    item_count = int(firstLine[0])
    capacity = int(firstLine[1])

    items = []

    for i in range(1, item_count+1):
        line = lines[i]
        parts = line.split()
        items.append(Item(i-1, int(parts[0]), int(parts[1])))

    # # a trivial algorithm for filling the knapsack
    # # it takes items in-order until the knapsack is full
    # value = 0
    # weight = 0
    # taken = [0]*len(items)

    # for item in items:
    #     if weight + item.weight <= capacity:
    #         taken[item.index] = 1
    #         value += item.value
    #         weight += item.weight
    
    value, opt, taken = knapsack_mip(item_count, items, capacity)
    # value, opt, taken = dynamic_model(item_count, items, capacity)

    # prepare the solution in the specified output format
    output_data = str(value) + ' ' + str(opt) + '\n'
    output_data += ' '.join(map(str, taken))
    return output_data


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/ks_4_0)')

