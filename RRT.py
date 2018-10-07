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
    parent_index = None

    # Node constructor
    # assign the x and y
    def __init__(self, x, y):

        self.x = x
        self.y = y

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

    searching_area = []

    expand_distance = None

    prob_to_search_around_goal = None

    """
    READ ME &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    class constructor
    start_pose: np array, start location
    goal_pose: np array, goal location
    obstacl_vertices_array: 2d np array, store the vertices of the obstacle
    searching_area: np array [x_min, x_max, y_min, y_max], searching area
    expand_distance: expansion distance
    prob_to_search_around_goalL the probability to search around the goal

    To do: assign the class variables

    How to build a new node

    mynode = node(x,y)

    """



    def __init__(self, start_pose, goal_pose, obstacl_vertices_array, searching_area, expand_distance = 1., prob_to_search_around_goal = 0.05 ):

        self.start_node = Node(start_pose[0],start_pose[1])
        self.goal_node = Node(goal_pose[0],goal_pose[1])
        self.obstacl_vertices_array = obstacl_vertices_array
        self.searching_area = searching_area

        self.expand_distance = expand_distance
        self.prob_to_search_around_goal = prob_to_search_around_goal
        # self.start_node is empty, need to call the node constructor

    def Planning(self):

        # push the start node into the node list
        self.node_list.append(self.start_node)
        self.start_node.parent_index = 0
        #while True:
        while True:
            if random.uniform(0, 1) < 0.95:
                x = random.uniform(self.searching_area[0],self.searching_area[1])
                y = random.uniform(self.searching_area[2],self.searching_area[3])
                sample_node = Node(x,y)
            else:
                sample_node = Node(self.goal_node.x, self.goal_node.y)

            NearestNode_index = self.GetNearestNodeIndex(sample_node)
            NearestNode = self.node_list[NearestNode_index]
            angle = np.arctan2(sample_node.x - NearestNode.x , sample_node.y - NearestNode.y)
            x_expand = NearestNode.x + np.cos(angle) * self.expand_distance
            y_expand = NearestNode.y + np.sin(angle) * self.expand_distance
            expand_node = Node(x_expand,y_expand)
            if not self.IsCollision(expand_node):
                expand_node.parent_index = NearestNode_index
                self.node_list.append(expand_node)
                expand_node_index =len(self.node_list) - 1
                #print expand_node.x , expand_node.y

                if self.IsGoalReached(expand_node):
                    self.goal_node.parent_index = expand_node_index
                    self.node_list.append(self.goal_node)
                    # reconstruct Path
                    path_node = self.goal_node
                    #self.path.append(path_node)
                    while path_node.parent_index:
                        self.path.append(path_node)
                        path_node_parent_index = path_node.parent_index
                        path_node = self.node_list[path_node_parent_index]
                        
                    self.path.append(self.start_node)

                    break

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

    def IsGoalReached(self, current_node):
        if ((current_node.x - self.goal_node.x)**2 + (current_node.y - self.goal_node.y)**2) <= 0.5:
            return True
        else:
            return False

    """
        given the current node pose, find the closest node index in the node_list
    """
    def GetNearestNodeIndex(self, sampled_node):
        global_min = 100000

        for index in range(0, len(self.node_list)):
            distance = (self.node_list[index].x - sampled_node.x)**2 + (self.node_list[index].y - sampled_node.y)**2
            if distance < global_min:
                global_min = distance
                index_min = index

        return index_min



    """
        given the current node position, check if collision occur
    """
    def IsCollision(self, current_node_pose):
        if current_node_pose.x > self.obstacl_vertices_array[0][0] and current_node_pose.x < self.obstacl_vertices_array[1][0] and current_node_pose.y > self.obstacl_vertices_array[0][1] and current_node_pose.y < self.obstacl_vertices_array[2][1]:
           return True
        else:
           return False




    """
        show the current state
    """


    def ShowPathState(self):
        #plt.clf()

        plt.figure()
        # plot the obstacle
        plt.plot(self.obstacl_vertices_array[:,0],self.obstacl_vertices_array[:,1], color = 'k')

        # connect every node in the node list with its parent
        for node in self.node_list:
            if node.parent_index:
                parent_node = self.node_list[node.parent_index]
                plt.plot([node.x, parent_node.x],[node.y, parent_node.y], "r-" )

        for index in range(len(self.path) - 1):

            plt.plot([self.path[index].x,self.path[index + 1].x],[self.path[index].y,self.path[index + 1].y],"b-")


        # plot the start position
        plt.plot(self.start_node.x, self.start_node.y, 'o', markersize=14, markeredgewidth=1,markeredgecolor='k', markerfacecolor='None' )
        # plot the goal position
        plt.plot(self.goal_node.x, self.goal_node.y, '*', markersize=14, markeredgewidth=1,markeredgecolor='k', markerfacecolor='None' )
        # plot the goal position




        plt.axis([-15, 15, -15, 15])

        plt.grid(True)

        plt.pause(0.01)
        plt.show()


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




def main():
    print("start simple RRT path planning")

    # ====Search Path with RRT====

    obstacl_vertices_array = np.array([[-4,-4], [4,-4], [4,4], [-4, 4],[-4,-4]])  # [x,y]


    # Set Initial parameters

    start_pose = np.array([-4,5])

    goal_pose = np.array([6, -7])

    searching_area = np.array([-10, 10, -10, 10])


    rrt = RRT_planner(start_pose, goal_pose, obstacl_vertices_array, searching_area)
    #rrt.ShowMap()

    rrt.Planning()
    rrt.ShowPathState()
if __name__ == '__main__':
    main()
