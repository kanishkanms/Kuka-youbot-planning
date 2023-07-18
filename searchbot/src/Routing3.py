# -*- coding: utf-8 -*-
"""
Created on Wed Apr 19 22:50:38 2023

@author: RAJBALAJ
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 22:33:58 2023

@author: RAJBALAJ
"""

import math
import itertools
import operator
import numpy as np

places = {
    0: [2,18],
    1: [10,10],
    2: [2,4],
    3: [12,18],
    4: [16,9],
    5: [18,12]
}


def dijkstra(problem):
    path ={}
    dist = [[[0 for i in range(len(problem.maze_map.map_data[0]))]for j in range(len(problem.maze_map.map_data))] for k in range(5)]
    for t in range(1,len(list(places.values()))-1):
        S0 = [places[t],0]
        fringe = []
        for i in range(len(problem.maze_map.map_data)):
            for j in range(len(problem.maze_map.map_data[0])):
                if [i,j] != list(S0[0]):
                    fringe.append([[i,j],math.inf])
                else:
                    fringe.append(S0)
        closed = []
        prev = {}
        def selectmin (d):
            n = 0
            min = d[0][1]
            for j in range(1,len(d)):
                if min > d[j][1]:
                    min = d[j][1]
                    n = j
                else:
                    continue
            return(d[n])
        
        while len(fringe) != 0:
            b = selectmin(fringe)
            if b[1] > 500:
                break
            dist[t][b[0][0]][b[0][1]] = b[1]
            fringe.remove(b) 
            closed.append(b)
            c = problem.getSuccessors(b[0])
            for element in c:
                count = 0
                f = len(fringe)
                element[2] = element[2] + b[1]
                for i in range(f):
                    if element[0] == fringe[i][0]:
                        count = 1
                        if fringe[i][1] > element[2]:
                            fringe.remove(fringe[i])
                            fringe.append([element[0],element[2]])
                            prev[tuple(element[0])] = b
                            break
                         
                if count == 1:
                    continue
                e = len(closed)
                for j in range(e):
                    if element[0] == closed[j][0]:
                        count = 2
                if count ==2:
                    continue
                prev[tuple(element[0])] = b

        for g in places.keys():
            sub_path = []
            a  = [places[g],0]
            if g!= t:
                while True:
                    sub_path.append(a[0])
                    a = prev[tuple(a[0])]
                    if places[t] == a[0]:
                        sub_path.append(a[0])
                        break
                path[str(g) + str(t)] = sub_path
    return([dist,path])


def Optimal(dist):
    places_list = list(places.keys())
    places_list.remove(0) 
    permutations1 = itertools.permutations(places_list)
    permutations2 = itertools.permutations(places_list+[5])
    permutations3 = itertools.permutations(places_list + [5,5])
    permutations4 = itertools.permutations(places_list + [5,5,5])
    permutations = []
    for path in permutations1:
        if path[0] == 5:
            continue
        if path[-1] == 5:
            continue
        permutations.append(path)
    for path in permutations2:
        breach = 0
        if path[0] == 5:
            continue
        for i in range(0,len(path)-1):
            if path[i] == path[i+1]:
                breach = 1
        if breach != 1:
            permutations.append(path)
    for path in permutations3:
        breach = 0
        if path[0] == 5:
            continue
        for i in range(0,len(path)-1):
            if path[i] == path[i+1]:
                breach = 1
        if breach != 1:
            permutations.append(path)
    for path in permutations4:
        breach = 0
        if path[0] == 5:
            continue
        for i in range(0,len(path)-1):
            if path[i] == path[i+1]:
                breach = 1
        if breach != 1:
            permutations.append(path)

    optimal_total_distance = math.inf
    optimal_path = None
    
    
    for path in permutations:
        total_distance = 0
        current_place = 0
        picked = 0
        for next_place in path:
            if picked == 3: 
                total_distance += dist[current_place][places[5][0]][places[5][1]]
                picked = 0
                current_place = next_place
                continue
            if next_place == 5: 
                total_distance += dist[current_place][places[5][0]][places[5][1]]
                picked = 0
                current_place = next_place
                continue
            if picked < 3: 
                total_distance += dist[next_place][places[current_place][0]][places[current_place][1]]
                picked += 1
                current_place = next_place
                continue
        if current_place != 5:
            total_distance += dist[current_place][places[5][0]][places[5][1]]
        
        if total_distance <  optimal_total_distance:
            optimal_total_distance = total_distance
            optimal_path = path
    optimal_path = [0] + list(optimal_path)
    if optimal_path[-1] != 5:
        optimal_path = optimal_path + [5]
        
    return(optimal_path)

def path_finder(problem,optimal_path,path):
    states_path = []
    for i in range(len(optimal_path)-1):
        if optimal_path[i+1] == 5:
            if i == len(optimal_path) - 2:
                e = path[str(optimal_path[i+1]) + str(optimal_path[i])]
                e.reverse()
                states_path.extend(e[1:])
                states_path.append("Drop")
            else:
                e = path[str(optimal_path[i+1]) + str(optimal_path[i])]
                e.reverse()
                states_path.extend(e[1:])
                states_path.append("Drop")
            continue
        if i == 0:
            e = path[str(optimal_path[i]) + str(optimal_path[i+1])]
            states_path.extend(e[1:])
            states_path.append("Pickup")

        else:
            e = path[str(optimal_path[i]) + str(optimal_path[i+1])]
            states_path.extend(e[1:])
            states_path.append("Pickup")
        
    return(states_path)       
        


    


