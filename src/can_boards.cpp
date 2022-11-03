#include "../include/can_boards.hpp"

CanBoards::CanBoards(ros::NodeHandle &node_handle) {
    this->can_boards.push_back(CanBoard(10));
    this->can_boards.push_back(CanBoard(11));
    this->can_boards.push_back(CanBoard(12));
}