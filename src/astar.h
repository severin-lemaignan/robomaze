#ifndef ASTAR_H
#define ASTAR_H

#include <memory> // for unique_ptr
#include <tuple>
#include <string>

typedef std::tuple<unsigned int, unsigned int> Position;

const Position START_POS {1, 1};
const Position END_GOAL {98, 98};

struct Node
{
    std::unique_ptr<Node> parent;
    Position position;
    unsigned int distance_to_start;
    unsigned int total_cost;
};

class AStar
{
public:
    AStar();

    Position planNextPosition();

    std::string getNextMove();

private:

    Position current_position;
};

#endif // ASTAR_H
