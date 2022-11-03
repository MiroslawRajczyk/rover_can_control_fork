#include "../include/test_can_boards.hpp"

CanBoards::CanBoards(ros::NodeHandle &node_handle) {
    can_boards.push_back(CanBoard(10));
    can_boards.push_back(CanBoard(11));
    can_boards.push_back(CanBoard(12));
    can_boards.push_back(CanBoard(13));

    for(int i=0; i < can_boards.size(); i++){
        th.push_back(std::thread(&CanBoard::worker, can_boards[i]));
        std::cout <<"Created worker thread: "<<th[i].get_id() <<std::endl;
    }
}

CanBoards::~CanBoards() {
    for(int i=0; i < can_boards.size(); i++){
        std::cout <<"Joining thread: "<<th[i].get_id() <<std::endl;
        th.at(i).join();
    }
}