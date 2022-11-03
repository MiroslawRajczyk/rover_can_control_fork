#pragma once

#include <ros/ros.h>
#include <vector>
#include <thread>

#include "test_can_board.hpp"

class CanBoards {
    private:
        std::vector<CanBoard> can_boards;
        std::vector<std::thread> th;
        std::vector<std::thread> some_threads;
    public:

    // usunąć wszystkie metody i dodać tylko wyszukiwarkę danego can_boarda zwracającego referencję na CanBoard
    //CanBoard* getCanBoard(unsigned int can_id);

    CanBoards(ros::NodeHandle &node_handle);
    ~CanBoards();
};