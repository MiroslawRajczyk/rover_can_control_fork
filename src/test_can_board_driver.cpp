#include <ros/ros.h>

#include "../include/test_can_boards.hpp"
#include "../include/test_manipulator_interface.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_can_board_driver");
    ros::NodeHandle node_handle;

    CanBoards can_boards(node_handle);
    //ManipulatorInterface manipulator_interface(node_handle, can_boards);
    ros::spin();

    return 0;
}
