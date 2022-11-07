#pragma once

#include <ros/ros.h>
#include "tools/CbVelocityArray.h"
#include "tools/CbEffortArray.h"
#include "tools/CbPositionArray.h"
#include "tools/PoseController.h"
#include "tools/CbPoseLimits.h"
#include "tools/CbEffortLimitsArray.h"
#include "tools/CbPositionLimitsArray.h"
#include "tools/encoder_set_offset.h"
#include "tools/cb_set_pid.h"
#include "tools/cb_set_pose_limits.h"
#include "tools/cb_set_effort_limits.h"
#include "tools/cb_set_frequency.h"
#include "std_msgs/String.h"

#include <vector>
#include <thread>
#include <string.h>
#include <fstream>

#include "test_can_board.hpp"
#include "../include/test_common.h"


class CanBoards {
    private:
        std::vector<CanBoard> can_boards;
        std::vector<std::thread> th;
        ros::Subscriber velocityGoalSubscriber, positionGoalSubscriber, effortGoalSubscriber;
        ros::ServiceServer setEncoderOffsetService, setEncoderPositionPidService,
            setEncoderVelocityPidService, setEncoderPositionLimitsService, setEncoderEffortLimitsService,
            setEncoderReadingsFrequencyService;
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
    void workerRosPublisher(ros::Publisher positionPub, ros::Publisher velocityPub, ros::Publisher effortPub, ros::Publisher positionLimitsPub, ros::Publisher effortLimitsPub);

    void readEncodersOffsetsFromFile(std::string path);
    bool setEncoderOffsetCallback(tools::encoder_set_offset::Request  &req, tools::encoder_set_offset::Response &res);
    bool setEncoderPositionPidCallback(tools::cb_set_pid::Request  &req, tools::cb_set_pid::Response &res);
    bool setEncoderVelocityPidCallback(tools::cb_set_pid::Request  &req, tools::cb_set_pid::Response &res);
    bool setEncoderPositionLimitsCallback(tools::cb_set_pose_limits::Request  &req, tools::cb_set_pose_limits::Response &res);
    bool setEncoderEffortLimitsCallback(tools::cb_set_effort_limits::Request  &req, tools::cb_set_effort_limits::Response &res);
    bool setEncoderReadingsFrequencyCallback(tools::cb_set_frequency::Request  &req, tools::cb_set_frequency::Response &res);
    void sendCanFrameRequest(int can_board_id, int can_frame_type);
};