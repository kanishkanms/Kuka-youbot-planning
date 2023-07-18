# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 22:33:17 2023

@author: RAJBALAJ
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 11:45:50 2023

@author: Bijo Sebastian
"""
import time
start_time = time.time()
import Routing
import copy
import Maps1
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
class Maze:
  """
  This class outlines the structure of the maze problem
  """
  
  maze_map = []# To store map data, start and goal points
  
  # Legal moves
  # [delta_x, delta_y, description]
  eight_neighbor_actions = {'up':[-1, 0], 'down':[1, 0], 'left': [0, -1], 'right': [0, 1],"upleft":[-1,-1],"upright":[-1,1],"downleft":[1,-1],"downright":[1,1]}
  
  #Setup plot
  map_plot_copy = []
  plot_colormap_norm = matplotlib.colors.Normalize(vmin=0.0, vmax=19.0)
  fig,ax = plt.subplots(1)
  plt.axis('equal')
  
  def plot_map(self,flag=True):
      """
      Plot
      """
      start = self.getStartState()
      goal = self.getGoalState()
      self.map_plot_copy[start[0]][start[1]] = Maps1.start_id
      self.map_plot_copy[goal[0]][goal[1]] = Maps1.goal_id
      plt.imshow(self.map_plot_copy, cmap=plt.cm.tab20c, norm=self.plot_colormap_norm)
      #plt.show()
      if(flag):
        plt.ion()
        plt.pause(0.00001)
      else:
        plt.ioff()
        plt.show()
  # default constructor
  def __init__(self, id):
      """
      Sets the map as defined in file Maps1
      """
      #Set up the map to be used
      self.maze_map = Maps1.maps_dictionary[id]
      self.map_plot_copy = copy.deepcopy(self.maze_map.map_data)
      #self.plot_map()
      return
     
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     start_state = self.maze_map.start
     return start_state
 
  def getGoalState(self):
     """
     Returns the start state for the search problem 
     """
     goal_state =  self.maze_map.goal
     return goal_state
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     if state == self.getGoalState():
         return True
     else:
         return False

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     #Update changes on the plot copy
     self.map_plot_copy[state[0]][state[1]] = Maps1.expanded_id
     
     successors = []
     for action in self.eight_neighbor_actions:
         
         #Get indiivdual action
         del_x, del_y = self.eight_neighbor_actions.get(action)
         
         #Get successor
         new_successor = [state[0] + del_x , state[1] + del_y]
         new_action = action
         
         # Check for obstacle 
         if self.maze_map.map_data[new_successor[0]][new_successor[1]] == Maps1.obstacle_id:
             continue
          
         #Update changes on the plot copy
         if self.map_plot_copy[new_successor[0]][new_successor[1]] != Maps1.expanded_id:
             self.map_plot_copy[new_successor[0]][new_successor[1]] = Maps1.fringe_id
         
         #Check cost
         if self.maze_map.map_data[new_successor[0]][new_successor[1]] == Maps1.free_space_id2:
             new_cost = Maps1.free_space_id2_cost
         elif action in ["upleft","upright","downleft","downright"]:
             new_cost = 1.414
         else: 
             new_cost = Maps1.free_space_id1_cost 
             
         successors.append([new_successor, new_action, new_cost])
         
     #Plot the changes
     #self.plot_map()
     return successors

def run_mobile_planning():
    current_maze = Maze(1)
    dist = Routing.dijkstra(current_maze)
    order = Routing.Optimal(dist)
    path = Routing.aStar(current_maze,order)
    
    
    if path:
        print('Found a path of %d moves: %s' % (len(path), str(path))) 
        #Display solution
        row,col = current_maze.getStartState() 
        path_states = []
        for action in path:
            del_x, del_y = current_maze.eight_neighbor_actions.get(action)
            newrow = row + del_x
            newcol = col + del_y
            path_states.append([newrow,newcol])
            #Update changes on the plot copy
            if [newrow , newcol] in list(Routing.places.values()):
                current_maze.map_plot_copy[newrow][newcol] = 4
            else:
                current_maze.map_plot_copy[newrow][newcol] = 10
            row = newrow
            col = newcol
            #current_maze.plot_map(False)

    else:
        print("Could not find a path")
    print("--- %s seconds ---" % (time.time() - start_time))
    Final_path_states = []
    for element in path_states:
        a = [element[0],element[1],1]
        R1 = [[0,1,-10],[-1,0,10],[0,0,1]]
        NC = np.matmul(R1,a)
        NC = NC/2
        Final_path_states.append([NC[0],NC[1]])
    print(Final_path_states)

    return Final_path_states
    

# if __name__ == '__main__':
    
#     current_maze = Maze(1)
#     dist = Routing.dijkstra(current_maze)
#     order = Routing.Optimal(dist)
#     path = Routing.aStar(current_maze,order)
    
    
#     if path:
#         print('Found a path of %d moves: %s' % (len(path), str(path))) 
#         #Display solution
#         row,col = current_maze.getStartState() 
#         path_states = []
#         for action in path:
#             del_x, del_y = current_maze.eight_neighbor_actions.get(action)
#             newrow = row + del_x
#             newcol = col + del_y
#             path_states.append([newrow,newcol])
#             #Update changes on the plot copy
#             if [newrow , newcol] in list(Routing.places.values()):
#                 current_maze.map_plot_copy[newrow][newcol] = 4
#             else:
#                 current_maze.map_plot_copy[newrow][newcol] = 10
#             row = newrow
#             col = newcol
#             #current_maze.plot_map(False)

#     else:
#         print("Could not find a path")
#     print("--- %s seconds ---" % (time.time() - start_time))
#     Final_path_states = []
#     for element in path_states:
#         a = [element[0],element[1],1]
#         R1 = [[0,1,-10],[-1,0,10],[0,0,1]]
#         NC = np.matmul(R1,a)
#         NC = NC/2
#         Final_path_states.append([NC[0],NC[1]])
#     print(Final_path_states)