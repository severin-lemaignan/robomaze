#include <climits>
#include <exception>
#include <iostream>

#include <stdint.h>
#include <map>

#include "astar.h"

using namespace std;

AStar::AStar() :
    current_position(START_POS) // initialiase current_pos to our start position
{
    maze.fill(false); // maze is initially unknown -> assume no obstacles
}

Position AStar::planNextPosition()
{
    // TODO: implement A* here!
    // Use 'maze' to find the shortest path from 'current_position' to END_GOAL
    // Return the next position to move to
    return current_position;
}

unsigned int AStar::heuristic(Position node, Position goal) {
    // TODO: implement a heuristic (eg squared Euclidean distance)
    return 0;
}

std::string AStar::getNextMove(Obstacle obstacles)
{
    // TODO: update the maze with new obstacle information,
    // then call planNextPosition() to get the next move
    return "S";
}
