#! /usr/bin/env python

import sys

import random
import time
import json
import requests

URL = "https://robomaze.skadge.org/api?"


MAZE_SIZE = (100, 100)
END_GOAL = (98,98)
START_POS = (1,1)


starttime = time.time()


# Based on https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = tuple(position)

        self.g = 0 # distance from start
        self.h = 0 # heuristic: estimated distance to goal
        self.f = 0 # total cost of the node

    def __eq__(self, other):
        return self.position == other.position

    def __repr__(self):
        return "Node: " + str((self.position, (self.g, self.h, self.f)))


class ProgressiveAstar:
    
    def __init__(self, start, end):

        self.x, self.y = start
        self.end = end

        self.maze = [0,] * MAZE_SIZE[0] * MAZE_SIZE[1] # initially, our maze is empty

    def astar(self, start):
        """Returns a list of tuples as a path from the given start to the given end in the given maze"""

        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, self.end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1] # Return reversed path

            # Generate children
            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                if node_position[0] > MAZE_SIZE[0] - 1 or \
                   node_position[0] < 0 or \
                   node_position[1] > MAZE_SIZE[1] -1 or \
                   node_position[1] < 0:
                    continue

                # Make sure walkable terrain
                if self.maze[node_position[1] * MAZE_SIZE[0] + node_position[0]] != 0:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                open_list.append(child)

            #print("%d nodes in closed_list, distance to goal: %d" % (len(closed_list), closed_list[-1].h))

    def get_next_move(self, obstacles):

        # update the maze
        # ["N", "S", "E", "W"]

        if obstacles[0]:
            self.maze[self.y * MAZE_SIZE[0] + self.x-1] = 1
        if obstacles[1]:
            self.maze[self.y * MAZE_SIZE[0] + self.x+1] = 1
        if obstacles[2]:
            self.maze[(self.y+1) * MAZE_SIZE[0] + self.x] = 1
        if obstacles[3]:
            self.maze[(self.y-1) * MAZE_SIZE[0] + self.x] = 1

        self.printmaze()

        # run A* with updated maze
        path = self.astar((self.x, self.y))

        #print(path)

        self.x += 1
        
        if path[1][0] - self.x == -1:
            self.x += 1
            return "E"
        elif path[1][0] - self.x == 1:
            self.x -= 1
            return "W"
        elif path[1][1] - self.y == -1:
            self.y += 1
            return "S"
        elif path[1][1] - self.y == 1:
            self.y -= 1
            return "N"
        else:
            raise Exception("Impossible move!")



    def printmaze(self):
        for j in range(MAZE_SIZE[1]):
            for i in range(MAZE_SIZE[0]):
                if self.maze[j * MAZE_SIZE[0] + i]:
                    sys.stdout.write(' ')
                else:
                    sys.stdout.write('#')
            sys.stdout.write('\n')



if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: random.py <name of robot>")
        sys.exit()

    name = sys.argv[1]

    astar = ProgressiveAstar(START_POS, END_GOAL)

    next_move = "E"

    try:
        while 1:
            response = requests.get(URL + "move=" + json.dumps([name, next_move]))

            obstacles = json.loads(response.text)[1]

            next_move = astar.get_next_move(obstacles)

            raw_input("Press enter for next step...")
            time.sleep(0.25)
    finally:
        print("Total elapsed time: %d seconds" %(time.time() - starttime))





