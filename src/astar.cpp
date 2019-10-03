#include <map>
#include <exception>
#include <limits>  // infinity
#include <stdlib.h>     // abs

#include "astar.h"

using namespace std;

AStar::AStar()
{
    maze.fill(false);

}

Position AStar::planNextPosition()
{

    map<Position, int> cost_to; // maps nodes to distance to 'start_node'
    cost_to[current_position] = 0;
    map<Position, Position> come_from; // needed to reconstruct shortest path

    NodesList nodes_to_visit = {{current_position,0}}; // (node, 'total' cost)

    while (!nodes_to_visit.empty()) {
        auto node = pop_best_node(nodes_to_visit);

        for (auto neighbour : neighbours(node)) {

            if ( cost_to.find(neighbour) == cost_to.end() ) // neighbour not known yet!
            {

                cost_to[neighbour] = numeric_limits<int>::max();
            }

            if (cost_to[node] + 1  < cost_to[neighbour]) // new shorter path to v!
            {
                cost_to[neighbour] = cost_to[node] + 1;
                nodes_to_visit.push_back({neighbour, cost_to[neighbour] + heuristic(neighbour, END_GOAL)});
                come_from[neighbour] = node;
            }
        }
    }

    // finally, reconstruct the path
    Position next_position = END_GOAL;

    while (true) {
        if (come_from[next_position] == current_position) break;
        next_position = come_from[next_position];
    }

    return next_position;

}

unsigned int AStar::heuristic(Position node, Position goal)
{
    int x,y, gx, gy;
    tie(x,y) = node;
    tie(gx,gy) = goal;
    return abs(x - gx) + abs(y - gy);
}

Position AStar::pop_best_node(NodesList& nodes)
{
    int best_node_score = numeric_limits<int>::max();
    size_t best_node_idx = 0;
    Position best;

    for (size_t idx = 0; idx < nodes.size(); idx++)
    {
        Position p; int score;
        tie(p, score) = nodes[idx];

        if (score < best_node_score) {
            best_node_score = score;
            best_node_idx = idx;
            best = p;
        }
    }

    nodes.erase(nodes.begin() + best_node_idx);
    return best;
}


bool AStar::isFree(Position node)
{
    int x, y;
    tie(x,y) = node;

    if (x < 0 || y < 0 ||
            x >= MAZE_SIZE || y >= MAZE_SIZE) {
        return false;
    }

    return !maze[y * MAZE_SIZE + x];
}

/** Get the list of (valid) neighbours
*/
vector<Position> AStar::neighbours(Position node)
{
    unsigned int x, y;
    tie(x,y) = node;

    vector<Position> nbours;

    if (isFree({x, y-1})) nbours.push_back({x, y-1});
    if (isFree({x, y+1})) nbours.push_back({x, y+1});
    if (isFree({x-1, y})) nbours.push_back({x-1, y});
    if (isFree({x+1, y})) nbours.push_back({x+1, y});

    return nbours;
}

std::string AStar::getNextMove(Obstacle obstacles)
{

    bool N, S, E, W;
    tie(N, S, E, W) = obstacles;

    int x, y;
    tie(x, y) = current_position; // unpack the tuple

    // first, update the maze
    if (N) 
        maze[(y-1) * MAZE_SIZE + x] = true;
    if (S)
        maze[(y+1) * MAZE_SIZE + x] = true;
    if (E)
        maze[y * MAZE_SIZE + x+1] = true;
    if (W)
        maze[y * MAZE_SIZE + x-1] = true;


    // run the A*
    unsigned nx, ny;
    auto next_position = planNextPosition();
    tie(nx, ny) = next_position;

    string direction;

    // update pos and return best move
    if (nx - x == 1) {
        x += 1;
        direction =  "E";
    }
    else if (nx - x == -1) {
        x -= 1;
        direction =  "W";
    }
    else if (ny - y == 1) {
        y += 1;
        direction =  "S";
    }
    else if (ny - y == -1) {
        y -= 1;
        direction =  "N";
    }
    else throw logic_error("Impossible move!");

    current_position = {x,y};

    return direction;
}
