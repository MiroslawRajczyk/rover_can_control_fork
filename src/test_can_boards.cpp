#include "../include/test_can_boards.hpp"

CanBoards::CanBoards() {
    can_boards.push_back(CanBoard(10));
    can_boards.push_back(CanBoard(11));
    can_boards.push_back(CanBoard(12));
    can_boards.push_back(CanBoard(13));
    can_boards.push_back(CanBoard(14));
    can_boards.push_back(CanBoard(15));
    
    this->readEncodersOffsetsFromFile("/home/miroslaw/manipulator_encoders_offsets.txt");

    for(int i=0; i < can_boards.size(); i++){
        th.push_back(std::thread(&CanBoard::workerCanSender, &can_boards[i]));
        std::cout <<"Created workerCanSender thread"<<std::endl;
        //th.push_back(std::thread(&CanBoard::workerCanReceiver, &can_boards[i]));
        //std::cout <<"Created workerCanRrecever thread"<<std::endl;
    }
    th.push_back(std::thread(&CanBoards::workerCanReceiver, this));
    

    ros::NodeHandle node_handler("can_board_driver");
    velocityGoalSubscriber = node_handler.subscribe("velocity_goal", 1, &CanBoards::velocityGoalSubscriberCallback, this);
    positionGoalSubscriber = node_handler.subscribe("position_goal", 1, &CanBoards::positionGoalSubscriberCallback, this);
    effortGoalSubscriber = node_handler.subscribe("effort_goal", 1, &CanBoards::effortGoalSubscriberCallback, this);
    ros::Publisher positionPublisher = node_handler.advertise<tools::PoseController>("position_real", 1);
    ros::Publisher velocityPublisher = node_handler.advertise<tools::CbVelocityArray>("velocity_real", 1);
    ros::Publisher effortPublisher = node_handler.advertise<tools::CbEffortArray>("effort_real", 1);
    setEncoderOffsetService = node_handler.advertiseService("set_encoder_offset", &CanBoards::setEncoderOffsetCallback, this);

    th.push_back(std::thread(&CanBoards::workerRosPublisher, this, positionPublisher, velocityPublisher, effortPublisher));
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
            if ((msg->velocity[i] <= 32.5) && (msg->velocity[i] >= -32.5)) {
                can_boards.at(i).setVelocityGoal(msg->velocity[i]);
                can_boards.at(i).setFrameType(12);
            } else {
                ROS_INFO("Received wrong value! Values in velocity_goal array need to in range from <-32.5 to 32.5>. Setting velocity to 0 for safety...");
                can_boards.at(i).setVelocityGoal(0);
            }
        }
    } else {
        ROS_INFO("Setting velocities of Can Boards require 6 values to be sent, received: %d", msg->velocity.size());
    }

}

void CanBoards::positionGoalSubscriberCallback(const tools::CbPositionArray::ConstPtr& msg) {
    if(msg->position.size() == 6) {
        for (int i = 0; i < msg->position.size();i++) {
            if ((msg->position[i] <= 180.0) && (msg->position[i] > -180.0)) {
                can_boards.at(i).setPositionGoal(msg->position[i]);
                can_boards.at(i).setFrameType(11);
            } else {
                ROS_INFO("Received wrong value! Values in position_goal array need to in range from (-180 to 180>. Setting position_goal to position_real for safety...");
                can_boards.at(i).setPositionGoal(can_boards.at(i).getPositionReal());
            }
        }
    } else {
        ROS_INFO("Setting position of Can Boards require 6 values to be sent, received: %d", msg->position.size());
    }

}

void CanBoards::effortGoalSubscriberCallback(const tools::CbEffortArray::ConstPtr& msg) {
    if(msg->effort.size() == 6) {
        for (int i =0; i< msg->effort.size();i++) {
            if ((msg->effort[i] <= 100) && (msg->effort[i] >= -100)) {
                can_boards.at(i).setEffortGoal(msg->effort[i]);
                can_boards.at(i).setFrameType(10);
            } else {
                ROS_INFO("Received wrong value! Values in effort_goal array need to in range from <-100 to 100>. Setting effort to 0 for safety...");
                can_boards.at(i).setEffortGoal(0);
            }
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
        can_boards[i].setEncoderOffset(std::stof(line));
        i++;
    }
}

void CanBoards::workerCanReceiver() {
    int s, i, id = -1;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct can_filter rfilter[6];

	rfilter[0].can_id   = 0x0A;
	rfilter[0].can_mask = 0xFFF;
    rfilter[1].can_id   = 0x0B;
	rfilter[1].can_mask = 0xFFF;
    rfilter[2].can_id   = 0x0C;
	rfilter[2].can_mask = 0xFFF;
    rfilter[3].can_id   = 0x0D;
	rfilter[3].can_mask = 0xFFF;
    rfilter[4].can_id   = 0x0E;
	rfilter[4].can_mask = 0xFFF;
    rfilter[5].can_id   = 0x0F;
	rfilter[5].can_mask = 0xFFF;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, "vcan0");
    ioctl(s, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        exit(1);
    }

    while (true)
    {
        if (s < 0)
        {
            std::cout << "Can socket error!" << std::endl;
        }
        else
        {
            setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
            nbytes = read(s, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                perror("Read");
            }
            // read can message with position, velocity and effort values from encoder-----
            if (frame.data[0] == 0x13) {
                switch(frame.can_id) {
                    case 0x0A: id = 0; break;
                    case 0x0B: id = 1; break;
                    case 0x0C: id = 2; break;
                    case 0x0D: id = 3; break;
                    case 0x0E: id = 4; break;
                    case 0x0F: id = 5; break;
                }
                can_boards.at(id).setPositionReal((((frame.data[1] << 8) | frame.data[2])*360.0/4096.0)-180.0);
                can_boards.at(id).setVelocityReal(((frame.data[3] << 8) | frame.data[4])/1000.0);
                if(can_boards.at(id).getVelocityReal() > 32.767) {
                    can_boards.at(id).setVelocityReal(can_boards.at(id).getVelocityReal() - 65.536);
                }
                can_boards.at(id).setEffortReal(frame.data[5]);
                if(can_boards.at(id).getEffortReal() > 150.0) {
                    can_boards.at(id).setEffortReal(can_boards.at(id).getEffortReal() - 256.0);
                }
                std::cout<<"Current Position: "<<can_boards.at(id).getPositionReal()<<" , velocity: "<<can_boards.at(id).getVelocityReal()<<" effort: "<<can_boards.at(id).getEffortReal()<<std::endl;
            }
            // ---------------------------------------------------------------------------
            printf("0x%03X [%d] ",frame.can_id, frame.can_dlc);
            for (i = 0; i < frame.can_dlc; i++)
                printf("%02X ",frame.data[i]);
            printf("\r\n");
        }
    }
    if (close(s) < 0) {
        perror("Close");
    }
}

void CanBoards::workerRosPublisher(ros::Publisher positionPub, ros::Publisher velocityPub, ros::Publisher effortPub) {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        tools::PoseController poseMsg;
        poseMsg.position = {can_boards.at(0).getPositionReal() + can_boards.at(0).getEncoderOffset(),can_boards.at(1).getPositionReal() + can_boards.at(1).getEncoderOffset(),can_boards.at(2).getPositionReal() + can_boards.at(2).getEncoderOffset(),can_boards.at(3).getPositionReal() + can_boards.at(3).getEncoderOffset(),can_boards.at(4).getPositionReal() + can_boards.at(4).getEncoderOffset() ,can_boards.at(5).getPositionReal() + can_boards.at(5).getEncoderOffset()};
        positionPub.publish(poseMsg);

        tools::CbVelocityArray velMsg;
        velMsg.velocity = {can_boards.at(0).getVelocityReal(),can_boards.at(1).getVelocityReal(),can_boards.at(2).getVelocityReal(),can_boards.at(3).getVelocityReal(),can_boards.at(4).getVelocityReal(),can_boards.at(5).getVelocityReal()};
        velocityPub.publish(velMsg);

        tools::CbEffortArray effMsg;
        effMsg.effort = {can_boards.at(0).getEffortReal(),can_boards.at(1).getEffortReal(),can_boards.at(2).getEffortReal(),can_boards.at(3).getEffortReal(),can_boards.at(4).getEffortReal(),can_boards.at(5).getEffortReal()};
        effortPub.publish(effMsg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool CanBoards::setEncoderOffsetCallback(tools::encoder_set_offset::Request  &req, tools::encoder_set_offset::Response &res) {
    can_boards[req.id].setEncoderOffset(req.new_value);
    if (can_boards[req.id].getEncoderOffset() == req.new_value) {
        std::fstream file;
        file.open("/home/miroslaw/manipulator_encoders_offsets.txt",std::ios_base::out);
        for(int i=0;i<can_boards.size();i++)
        {
            file << can_boards[i].getEncoderOffset() << std::endl;
        }
        file.close();
        res.success = true;
        return true;
    }
}
