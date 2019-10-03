#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds

#include "astar.h"

using namespace std;
using namespace std::chrono_literals;

AStar::AStar()
{

}

Position AStar::planNextPosition()
{

}

std::string AStar::getNextMove()
{

    this_thread::sleep_for(250ms);

    return "S";

}
