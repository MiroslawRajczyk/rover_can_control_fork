#pragma once

#include <ros/ros.h>
#include "tools/CbVelocityArray.h"
#include "tools/CbEffortArray.h"
#include "tools/CbPositionArray.h"
#include "tools/PoseController.h"
#include "tools/encoder_set_offset.h"
#include "tools/cb_set_pid.h"

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
        ros::ServiceServer setEncoderOffsetService, setEncoderPositionPidService, setEncoderVelocityPidService;
    public:
    CanBoards();
    ~CanBoards();
    //CanBoard getCanBoardFromId(int id);
    void setCanBoardCanId(int index, int new_id);
    void disableCanBoardsThreads();
    void velocityGoalSubscriberCallback(const tools::CbVelocityArray::ConstPtr& msg);
    void positionGoalSubscriberCallback(const tools::CbPositionArray::ConstPtr& msg);
    void effortGoalSubscriberCallback(const tools::CbEffortArray::ConstPtr& msg);
    void workerCanReceiver();
    void workerRosPublisher(ros::Publisher positionPub, ros::Publisher velocityPub, ros::Publisher effortPub);

    void readEncodersOffsetsFromFile(std::string path);
    bool setEncoderOffsetCallback(tools::encoder_set_offset::Request  &req, tools::encoder_set_offset::Response &res);
    bool setEncoderPositionPidCallback(tools::cb_set_pid::Request  &req, tools::cb_set_pid::Response &res);
    bool setEncoderVelocityPidCallback(tools::cb_set_pid::Request  &req, tools::cb_set_pid::Response &res);

};