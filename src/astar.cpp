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
    map<Position, unsigned int> cost_to = {}; // maps nodes to distance to 'start_node'
    cost_to[current_position] = 0;

    map<Position, Position> come_from = {}; // needed to reconstruct shortest path

    NodesList nodes_to_visit = {{current_position,0}}; // (node, 'total' cost)

    while (!nodes_to_visit.empty()) {

        auto node = pop_best_node(nodes_to_visit);

        if (node == END_GOAL) break;

        for (auto neighbour : neighbours(node)) {

            if (cost_to.find(neighbour) == cost_to.end()){ // neighbour not seen yet -> cost +inf
                cost_to[neighbour] = UINT_MAX;
            }

            if (cost_to[node] + 1  < cost_to[neighbour]) { // new shorter path to neighbour!
                cost_to[neighbour] = cost_to[node] + 1;
                nodes_to_visit.push_back({neighbour, cost_to[neighbour] + heuristic(neighbour, END_GOAL)});
                come_from[neighbour] = node;
            }
        }
    }

    // finally, reconstruct the path
    vector<Position> path;
    Position node = END_GOAL;
    while(true) {

        path.push_back(node); // append at the front of our path
        node = come_from[node];
        if (node == current_position) break;
    }
    return path.back();

}


unsigned int AStar::heuristic(Position node, Position goal) {

    int x,y,gx,gy;
    tie(x,y) = node;
    tie(gx, gy) = goal;

    // use a Euclidian distance to get to the goal in straight line
    return (x - gx)*(x-gx) + (y - gy)*(y-gy);
}

Position AStar::pop_best_node(NodesList& nodes) { // we modify the list of nodes -> need to be passed by reference

    unsigned int best_node_score = UINT_MAX;
    size_t best_node_idx = 0;

    Position best;

    for (size_t idx = 0; idx < nodes.size(); idx++) {

        Position position; int score;
        tie(position, score) = nodes[idx];

        if (score < best_node_score) {
            best_node_score = score;
            best_node_idx = idx;
            best = position;
        }
    }

    nodes.erase(nodes.begin() + best_node_idx);

    return best;
}

bool AStar::is_free(Position node) {

    int x,y;

    tie(x, y) = node;

    if (x < 0 || y < 0 ||
            x >= MAZE_SIZE || y >= MAZE_SIZE) {
        return false;
    }

    return !maze[y * MAZE_SIZE + x];
}

vector<Position> AStar::neighbours(Position node) {

    int x, y;
    tie(x,y) = node;

    vector<Position> nbours;

    if (is_free({x, y-1})) nbours.push_back({x, y-1});
    if (is_free({x, y+1})) nbours.push_back({x, y+1});
    if (is_free({x-1, y})) nbours.push_back({x-1, y});
    if (is_free({x+1, y})) nbours.push_back({x+1, y});

    return nbours;

}


std::string AStar::getNextMove(Obstacle obstacles)
{

    // update the maze

    int x,y;
    bool N,S,E,W;

    tie(x, y) = current_position;
    tie(N,S,E,W) = obstacles;

    if (N) maze[(y-1) * MAZE_SIZE + x] = true;
    if (S) maze[(y+1) * MAZE_SIZE + x] = true;
    if (E) maze[y * MAZE_SIZE + x+1] = true;
    if (W) maze[y * MAZE_SIZE + x-1] = true;

    // run the A*
    auto next_position = planNextPosition();

    int nx, ny;
    tie(nx, ny) = next_position;

    string direction = "";

    // update pos and return best move
    if (nx - x == 1) {
        current_position = {x + 1,y};
        direction =  "E";
    }
    else if (nx - x == -1) {
        current_position = {x - 1,y};
        direction =  "W";
    }
    else if (ny - y == 1) {
        current_position = {x, y + 1};
        direction =  "S";
    }
    else if (ny - y == -1) {
        current_position = {x, y - 1};
        direction =  "N";
    }
    else throw logic_error("Impossible move!");

    return direction;

}
