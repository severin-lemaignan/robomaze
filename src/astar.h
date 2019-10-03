#ifndef ASTAR_H
#define ASTAR_H

#include <memory> // for unique_ptr
#include <tuple>
#include <vector>
#include <array>
#include <string>

typedef std::tuple<int, int> Position;
typedef std::tuple<Position, int> Node; // {position, score}
typedef std::vector<Node> NodesList;
typedef std::tuple<bool, bool, bool, bool> Obstacle;

const int MAZE_SIZE = 10;
const Position START_POS {1, 1};
const Position END_GOAL {98, 98};

class AStar
{
public:
    AStar();

    Position planNextPosition();

    std::string getNextMove(Obstacle obstacle);

private:

    // contains our maze (the portion we have already explored, anyway)
    // 'true' means 'there is a wall'
    std::array<bool, MAZE_SIZE * MAZE_SIZE> maze;

    Position current_position;

    unsigned int heuristic(Position node, Position goal);
    Position pop_best_node(NodesList& nodes);
    bool isFree(Position node);
    std::vector<Position> neighbours(Position node);
};

#endif // ASTAR_H
