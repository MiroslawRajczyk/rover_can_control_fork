#include <ros/ros.h>
#include <signal.h>

#include "../include/test_can_boards.hpp"
#include "../include/test_manipulator_interface.hpp"
#include "../include/test_common.h"

bool enableThreads = true;
void sigIntHandler(int s){
    enableThreads = false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_can_board_driver");
    CanBoards boards;

    signal (SIGINT,sigIntHandler);
    //ManipulatorInterface manipulator_interface(node_handle, can_boards);
    while(true) {
        ros::spinOnce();
        if (!enableThreads) {
            boards.disableCanBoardsThreads();
            exit (1);
        }
    }
    return 0;
}
