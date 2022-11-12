#include <ros/ros.h>
#include <signal.h>

#include "../include/manipulator_can_boards.hpp"
#include "../include/manipulator_common.h"

bool enableThreads = true;
void sigIntHandler(int s){
    enableThreads = false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "manipulator_can_board_driver");
    CanBoards boards;

    signal (SIGINT,sigIntHandler);
    while(true) {
        ros::spinOnce();
        if (!enableThreads) {
            boards.disableCanBoardsThreads();
            exit (1);
        }
    }
    return 0;
}
