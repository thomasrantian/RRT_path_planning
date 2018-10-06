# -*- coding: utf-8 -*-
"""
Created on Sat Oct  6 12:41:09 2018

@author: tianr
"""


"""
Import the packages

"""
import matplotlib.pyplot as plt
import random
import math
import copy
import numpy as np


show_animation = True

"""
Node class, (x,y) is the current node possition, paraent_index is the index of its paraent 
in the node list
"""

class Node():
    x = None
    y = None
    paraent_index = None
    
    # Node constructor
    # assign the x and y
    def __init__(self, x, y):
        pass
    
"""
RRT planner class, use RRT path planning method to plan the motion

"""


class RRT_planner():
    
   
    
    obstacl_vertices_array = None
    
    node_list = None
    
    start_node = None
    
    goal_node = None
    
    node_list = []
    
    path = []
    
    
    """
    class constructor
    start_pose: np array, start location
    goal_pose: np array, goal location
    obstacl_vertices_array: 2d np array, store the vertices of the obstacle
    searching_area: np array [x_min, x_max, y_min, y_max], searching area
    expand_distance: expansion distance
    prob_to_search_around_goalL the probability to search around the goal
    
    
    To do: assign the class variables
    
    
    """
    
    def __int__(self, start_pose, goal_pose, obstacl_vertices_array,
                searching_area, expand_distance = 1., prob_to_search_around_goal = 0.05 ):
        pass
    
    def Planning(self):
        
        # push the start node into the node list
        self.node_list.append(self.start_node)
        
        while True:
            
            """
            
            1. randomly sampling a point
                a. with probability (1-prob_to_search_around_goal), sample a point in the searching_area
                b. with probability prob_to_search_around_goal, set the sampled point to be the goal point
            2. select the nearest node from the node list, expand the nearest node to generate the current node
            
            3. check if the expanded current node is in the obstacle region
            
            4. if the current node checked out, assign parent index, contineue to search
            
            5. check goal, if the distance between the current node and the goal node is smaller than 0.5 m, break
               add the goal node to the node list
            
            """
        """
        back tracking, construct the path
        """
            
    def IsGoalReached(self, current)
        
        
    """
        given the current node pose, find the closest node index in the node_list
    """    
    def GetNearestNodeIndex(self, sampled_node_pose):
        pass
    
    
    
    """
        given the current node position, check if collision occur
    """
    def IsCollision(self, current_node_pose):
        pass
    
    
    """
        show the current state
    """
    
    
    def ShowPathState(self):
        plt.clf()
        
        # plot the obstacle
        plt.plot(self.obstacl_vertices_array[:,0],self.obstacl_vertices_array[:,1], color = 'k')
        
        # connect every node in the node list with its parent
        for node in self.node_list:
            if node.parent_index:
                plt.plot([node.x, node.y],[node_list[node.parent_index].x, node_list[node.parent_index].y], "r-" )
        # plot the start position
        plt.plot(self.start_node.x, self.start_node.y, 'o', markersize=14, markeredgewidth=1,markeredgecolor='k', markerfacecolor='None' )
        # plot the goal position
        plt.plot(self.goal_node.x, self.goal_node.y, '*', markersize=14, markeredgewidth=1,markeredgecolor='k', markerfacecolor='None' )
        # plot the goal position
        
        plt.axis([-15, 15, -15, 15])
        
        plt.grid(True)
        
        plt.pause(0.01)
        
    
    def ShowMap(self):
        
        plt.figure()
        # plot the obstacle
        plt.plot(self.obstacl_vertices_array[:,0],self.obstacl_vertices_array[:,1], color = 'k')
        plt.axes().set_aspect('equal') # Scale the plot size to get same aspect ratio
        plt.axis([-15, 15, -15, 15])
        plt.grid(True)
        
        # plot the start position
        plt.plot(self.start_node.x, self.start_node.y, 'o', markersize=14, markeredgewidth=1,markeredgecolor='k', markerfacecolor='None' )
        # plot the goal position
        plt.plot(self.goal_node.x, self.goal_node.y, '*', markersize=14, markeredgewidth=1,markeredgecolor='k', markerfacecolor='None' )
        # plot the goal position
        plt.show()
    
    
    def DrawPath(self):
        pass
        
        


def main():
    print("start simple RRT path planning")

    # ====Search Path with RRT====
    
    obstacl_vertices_array = np.array([[4,4],[4,-4],[-4,-4],[-4,4], [4,4]])  # [x,y]
    
    
    # Set Initial parameters
    
    start_pose = np.array([-4,5])
    
    goal_pose = np.array([6, -7])
    
    searching_area = np.array([-10, 10, -10, 10])
    
    
    rrt = RRT_planner(start_pose, goal_pose, obstacl_vertices_array, searching_area)
    rrt.ShowMap()
    

if __name__ == '__main__':
    main()