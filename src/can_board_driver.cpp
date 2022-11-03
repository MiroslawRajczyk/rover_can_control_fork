#include <ros/ros.h>

#include "../include/can_boards.hpp"
#include "../include/manipulator_interface.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "can_board_driver");
    ros::NodeHandle node_handle;

    //std::unique_ptr<CanBoards> can_boards(new int(15));

    CanBoards can_boards(node_handle);
    //ManipulatorInterface manipulator_interface(node_handle, can_boards);

    ros::spin();

    return 0;
}
