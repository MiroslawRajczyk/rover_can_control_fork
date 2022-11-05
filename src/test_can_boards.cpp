#include "../include/test_can_boards.hpp"

CanBoards::CanBoards() {
    can_boards.push_back(CanBoard(10));
    can_boards.push_back(CanBoard(11));
    can_boards.push_back(CanBoard(12));
    can_boards.push_back(CanBoard(13));
    can_boards.push_back(CanBoard(14));
    can_boards.push_back(CanBoard(15));
    
    this->readEncodersOffsetsFromFile("/home/nvidia/manipulator_encoders_offsets.txt");

    for(int i=0; i < can_boards.size(); i++){
        th.push_back(std::thread(&CanBoard::workerCanSender, &can_boards[i]));
        std::cout <<"Created workerCanSender thread"<<std::endl;
        //th.push_back(std::thread(&CanBoard::workerCanReceiver, &can_boards[i]));
        //std::cout <<"Created workerCanRrecever thread"<<std::endl;
    }
    th.push_back(std::thread(&CanBoard::workerCanReceiver, &can_boards[0]));

    ros::NodeHandle node_handler("can_board_driver");
    velocityGoalSubscriber = node_handler.subscribe("velocity_goal", 1, &CanBoards::velocityGoalSubscriberCallback, this);
    positionGoalSubscriber = node_handler.subscribe("position_goal", 1, &CanBoards::positionGoalSubscriberCallback, this);
    effortGoalSubscriber = node_handler.subscribe("effort_goal", 1, &CanBoards::effortGoalSubscriberCallback, this);
}

CanBoards::~CanBoards() {
    for(int i=0; i < can_boards.size(); i++){
        std::cout <<"Joining thread: "<<th[i].get_id() <<std::endl;
        th.at(i).join();
    }
}

void CanBoards::setCanBoardCanId(int index, int new_id) {
    can_boards.at(index).setCanId(new_id);
}

void CanBoards::disableCanBoardsThreads() {
    for(int i=0; i < can_boards.size(); i++){
        can_boards.at(i).setIsEnabled(false);
    }
}

void CanBoards::velocityGoalSubscriberCallback(const tools::CbVelocityArray::ConstPtr& msg) {
    if(msg->velocity.size() == 6) {
        for (int i =0; i< msg->velocity.size();i++) {
            can_boards.at(i).setVelocityGoal(msg->velocity[i]);
            can_boards.at(i).setFrameType(12);
        }
    } else {
        ROS_INFO("Setting velocities of Can Boards require 6 values to be sent, received: %d", msg->velocity.size());
    }

}

void CanBoards::positionGoalSubscriberCallback(const tools::CbPositionArray::ConstPtr& msg) {
    if(msg->position.size() == 6) {
        for (int i =0; i< msg->position.size();i++) {
            can_boards.at(i).setPositionGoal(msg->position[i]);
            can_boards.at(i).setFrameType(11);

        }
    } else {
        ROS_INFO("Setting position of Can Boards require 6 values to be sent, received: %d", msg->position.size());
    }

}

void CanBoards::effortGoalSubscriberCallback(const tools::CbEffortArray::ConstPtr& msg) {
    if(msg->effort.size() == 6) {
        for (int i =0; i< msg->effort.size();i++) {
            can_boards.at(i).setEffortGoal(msg->effort[i]);
            can_boards.at(i).setFrameType(10);
        }
    } else {
        ROS_INFO("Setting effort of Can Boards require 6 values to be sent, received: %d", msg->effort.size());
    }

}

void CanBoards::readEncodersOffsetsFromFile(std::string path) {
    std::string line;    
    std::ifstream fin(path);
    int i = 0;
    while(getline(fin,line)){
        can_boards.at(i).setEncoderOffset(std::stof(line));
        i++;
    }
}
