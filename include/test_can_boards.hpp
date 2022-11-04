#pragma once

#include <ros/ros.h>
#include "tools/CbVelocityArray.h"
#include "tools/CbEffortArray.h"
#include "tools/CbPositionArray.h"


#include "std_msgs/String.h"

#include <vector>
#include <thread>
#include <string.h>
#include <fstream>

#include "test_can_board.hpp"


class CanBoards {
    private:
        std::vector<CanBoard> can_boards;
        std::vector<std::thread> th;
        ros::Subscriber velocityGoalSubscriber, positionGoalSubscriber, effortGoalSubscriber;
    public:
    CanBoards();
    ~CanBoards();
    //CanBoard getCanBoardFromId(int id);
    void setCanBoardCanId(int index, int new_id);
    void disableCanBoardsThreads();
    void velocityGoalSubscriberCallback(const tools::CbVelocityArray::ConstPtr& msg);
    void positionGoalSubscriberCallback(const tools::CbPositionArray::ConstPtr& msg);
    void effortGoalSubscriberCallback(const tools::CbEffortArray::ConstPtr& msg);

    void readEncodersOffsetsFromFile(std::string path);
};