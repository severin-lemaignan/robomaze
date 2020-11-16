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
}

unsigned int AStar::heuristic(Position node, Position goal) {
}

std::string AStar::getNextMove(Obstacle obstacles)
{
    return "S";
}
