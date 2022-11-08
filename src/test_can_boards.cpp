#include "../include/test_can_boards.hpp"

const char * globalCanInterface;
const char * globaloffsetsFilePath;
int globalLoopback;

CanBoards::CanBoards() {
    std::string tmpCanInterface, tmpOffsetsFilePath;
    ros::NodeHandle node_handler("can_board_driver");
    // Loading can_interface param from launchfile
    if (node_handler.hasParam("/test_can_board_driver/can_interface")) {
        node_handler.getParam("/test_can_board_driver/can_interface", tmpCanInterface);
        globalCanInterface = tmpCanInterface.c_str();
        ROS_INFO("Using CAN interface: %s", globalCanInterface);
    } else {
        ROS_INFO("No param 'can_interface' stated! Using CAN interface: can0.");
        globalCanInterface = "can0";
    }
    // Loading enable_can_loopback param from launchfile
    if (node_handler.hasParam("/test_can_board_driver/enable_can_loopback")) {
        node_handler.getParam("/test_can_board_driver/enable_can_loopback", globalLoopback);
        ROS_INFO("Enable CAN loopback: %d", globalLoopback);
    } else {
        ROS_INFO("No param 'enable_can_loopback' stated! Using enable CAN loopback: 0.");
        globalLoopback = 0;
    }
    // Loading offsets_file_path param from launchfile
    if (node_handler.hasParam("/test_can_board_driver/offsets_file_path")) {
        node_handler.getParam("/test_can_board_driver/offsets_file_path", tmpOffsetsFilePath);
        globaloffsetsFilePath = tmpOffsetsFilePath.c_str();
        ROS_INFO("Encoders offsets file path: %s", globaloffsetsFilePath);
    } else {
        ROS_INFO("No param 'offsets_file_path' stated! Using encoders offsets file path: /home/nvidia/manipulator_encoders_offsets.txt.");
        globaloffsetsFilePath = "/home/nvidia/manipulator_encoders_offsets.txt";
    }
    canBoardsCanInterface = globalCanInterface;


    can_boards.push_back(CanBoard(10));
    can_boards.push_back(CanBoard(11));
    can_boards.push_back(CanBoard(12));
    can_boards.push_back(CanBoard(13));
    can_boards.push_back(CanBoard(14));
    can_boards.push_back(CanBoard(15));

    this->readEncodersOffsetsFromFile(globaloffsetsFilePath);
    // Add CAN receiver thread
    th.push_back(std::thread(&CanBoards::workerCanReceiver, this));
    for(int i=0; i < can_boards.size(); i++){
        // Add CAN sender thread for can board
        th.push_back(std::thread(&CanBoard::workerCanSender, &can_boards[i]));
        ROS_INFO("Created can sender thread for CanBoard %d", i);
        // Request position PID for can board
        sendCanFrameRequest(can_boards.at(i).getCanId(),0x18);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // Request velocity PID for can board
        sendCanFrameRequest(can_boards.at(i).getCanId(),0x19);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // Request position limits from can board
        sendCanFrameRequest(can_boards.at(i).getCanId(),0x1A);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // Request effort limits from can board
        sendCanFrameRequest(can_boards.at(i).getCanId(),0x1B);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // Request encoders readings CAN Frames frequency can board
        sendCanFrameRequest(can_boards.at(i).getCanId(),0x1C);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    velocityGoalSubscriber = node_handler.subscribe("set_velocity_goal", 1, &CanBoards::velocityGoalSubscriberCallback, this);
    positionGoalSubscriber = node_handler.subscribe("set_position_goal", 1, &CanBoards::positionGoalSubscriberCallback, this);
    effortGoalSubscriber = node_handler.subscribe("set_effort_goal", 1, &CanBoards::effortGoalSubscriberCallback, this);
    ros::Publisher positionPublisher = node_handler.advertise<tools::PoseController>("get_position_real", 1);
    ros::Publisher velocityPublisher = node_handler.advertise<tools::CbVelocityArray>("get_velocity_real", 1);
    ros::Publisher effortPublisher = node_handler.advertise<tools::CbEffortArray>("get_effort_real", 1);
    ros::Publisher positionPidsPublisher = node_handler.advertise<tools::CbPidArray>("get_position_pids", 1);
    ros::Publisher velocityPidsPublisher = node_handler.advertise<tools::CbPidArray>("get_velocity_pids", 1);
    ros::Publisher positionLimitsPublisher = node_handler.advertise<tools::CbPositionLimitsArray>("get_position_limits", 1);
    ros::Publisher effortLimitsPublisher = node_handler.advertise<tools::CbEffortLimitsArray>("get_effort_limits", 1);
    ros::Publisher encoderOffsetPublisher = node_handler.advertise<tools::CbOffsetArray>("get_encoders_offsets", 1);
    ros::Publisher readingsFrequencyPublisher = node_handler.advertise<tools::CbFrequencyArray>("get_readings_frequencies", 1);
    setEncoderOffsetService = node_handler.advertiseService("set_encoder_offset", &CanBoards::setEncoderOffsetCallback, this);
    setEncoderPositionPidService = node_handler.advertiseService("set_position_pid", &CanBoards::setEncoderPositionPidCallback, this);
    setEncoderVelocityPidService = node_handler.advertiseService("set_velocity_pid", &CanBoards::setEncoderVelocityPidCallback, this);
    setEncoderPositionLimitsService = node_handler.advertiseService("set_position_limits", &CanBoards::setEncoderPositionLimitsCallback, this);
    setEncoderEffortLimitsService = node_handler.advertiseService("set_effort_limits", &CanBoards::setEncoderEffortLimitsCallback, this);
    setEncoderReadingsFrequencyService = node_handler.advertiseService("set_readings_frequency", &CanBoards::setEncoderReadingsFrequencyCallback, this);
    th.push_back(std::thread(&CanBoards::workerRosPublisher, this, positionPublisher, velocityPublisher, effortPublisher, positionLimitsPublisher, effortLimitsPublisher, positionPidsPublisher, velocityPidsPublisher, encoderOffsetPublisher, readingsFrequencyPublisher));
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
            if ((msg->position[i] < 180.0) && (msg->position[i] >= -180.0)) {
                float tmp_goal = msg->position[i] - can_boards.at(i).getEncoderOffset();
                if (tmp_goal < -180.0) {
                    tmp_goal+=360.0;
                    } else if (tmp_goal >= 180.0) {
                        tmp_goal-=360.0;
                    }
                can_boards.at(i).setPositionGoal(tmp_goal);
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
    strcpy(ifr.ifr_name, globalCanInterface);
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
            switch(frame.can_id) {
                case 0x0A: id = 0; break;
                case 0x0B: id = 1; break;
                case 0x0C: id = 2; break;
                case 0x0D: id = 3; break;
                case 0x0E: id = 4; break;
                case 0x0F: id = 5; break;
            }
            // read can message with position, velocity and effort values from encoder----
            if (frame.data[0] == 0x13) {
                can_boards.at(id).setPositionReal((((frame.data[1] << 8) | frame.data[2])*360.0/4096.0)-180.0);
                if(can_boards.at(id).getPositionReal() < -180.0) {
                    can_boards.at(id).setPositionReal(can_boards.at(id).getPositionReal() + 360.0);
                } else if(can_boards.at(id).getPositionReal() >= 180.0) {
                    can_boards.at(id).setPositionReal(can_boards.at(id).getPositionReal() - 360.0);
                }
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
            // read can message with position PID from encoder ---------------------------
            if (frame.data[0] == 0x18) {
                can_boards.at(id).setPositionPID(((frame.data[1] << 8) | frame.data[2])/1000.0, ((frame.data[3] << 8) | frame.data[4])/1000.0, ((frame.data[5] << 8) | frame.data[6])/1000.0);
                std::cout <<"CanBoard "<<id<<" position PID: p: "<<can_boards.at(id).getPositionPID().p<<", i: "<<can_boards.at(id).getPositionPID().i<<", d: "<<can_boards.at(id).getPositionPID().d<<std::endl;
            }
            // ---------------------------------------------------------------------------
            // read can message with position PID from encoder ---------------------------
            if (frame.data[0] == 0x19) {
                can_boards.at(id).setVelocityPID(((frame.data[1] << 8) | frame.data[2])/1000.0, ((frame.data[3] << 8) | frame.data[4])/1000.0, ((frame.data[5] << 8) | frame.data[6])/1000.0);
                std::cout <<"CanBoard "<<id<<" velocity PID: p: "<<can_boards.at(id).getVelocityPID().p<<", i: "<<can_boards.at(id).getVelocityPID().i<<", d: "<<can_boards.at(id).getVelocityPID().d<<std::endl;
            }
            // ---------------------------------------------------------------------------
            // read can message with position limits from encoder ------------------------
            if (frame.data[0] == 0x1A) {
                can_boards.at(id).setPositionLimits((((frame.data[1] << 8) | frame.data[2])*360.0/4096.0)-180.0, (((frame.data[3] << 8) | frame.data[4])*360.0/4096.0-180));
                std::cout <<"CanBoard "<<id<<" position limits: from:"<<can_boards.at(id).getPositionLimits().from<<", to: "<<can_boards.at(id).getPositionLimits().to<<std::endl;
            }
            // ---------------------------------------------------------------------------
            // read can message with effort limits from encoder --------------------------
            if (frame.data[0] == 0x1B) {
                can_boards.at(id).setEffortLimits(frame.data[1], frame.data[2]);
                std::cout <<"CanBoard "<<id<<" effort limits: min:"<<can_boards.at(id).getEffortLimits().min<<", max: "<<can_boards.at(id).getEffortLimits().max<<std::endl;
            }
            // ---------------------------------------------------------------------------
            // read can message with readings frequency from encoder ---------------------
            if (frame.data[0] == 0x1C) {
                can_boards.at(id).setEncoderReadingsFrequency(100/frame.data[1]);
                std::cout <<"CanBoard "<<id<<" readings frequency: "<<can_boards.at(id).getEncoderReadingsFrequency()<<std::endl;
            }
            // ---------------------------------------------------------------------------
            //printf("0x%03X [%d] ",frame.can_id, frame.can_dlc);
            //for (i = 0; i < frame.can_dlc; i++)
            //    printf("%02X ",frame.data[i]);
            //printf("\r\n");
        }
    }
    if (close(s) < 0) {
        perror("Close");
    }
}

void CanBoards::workerRosPublisher(ros::Publisher positionPub, ros::Publisher velocityPub, ros::Publisher effortPub, ros::Publisher positionLimitsPub, ros::Publisher effortLimitsPub, ros::Publisher positionPidsPublisher, ros::Publisher velocityPidsPublisher, ros::Publisher encoderOffsetPublisher, ros::Publisher readingsFrequencyPublisher) {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        tools::PoseController poseMsg;
        float tmp_position;
        for(int i = 0; i <can_boards.size(); i++) {
            if ((can_boards.at(i).getPositionReal() + can_boards.at(0).getEncoderOffset()) < -180.0) {
                poseMsg.position.push_back(can_boards.at(i).getPositionReal() + can_boards.at(i).getEncoderOffset() + 360.0);
            } else if ((can_boards.at(i).getPositionReal() + can_boards.at(0).getEncoderOffset()) >= 180.0) {
                poseMsg.position.push_back(can_boards.at(i).getPositionReal() + can_boards.at(i).getEncoderOffset() - 360.0);
            } else {
                poseMsg.position.push_back(can_boards.at(i).getPositionReal() + can_boards.at(i).getEncoderOffset());
            }
        }
        positionPub.publish(poseMsg);

        tools::CbVelocityArray velMsg;
        velMsg.velocity = {can_boards.at(0).getVelocityReal(),can_boards.at(1).getVelocityReal(),can_boards.at(2).getVelocityReal(),can_boards.at(3).getVelocityReal(),can_boards.at(4).getVelocityReal(),can_boards.at(5).getVelocityReal()};
        velocityPub.publish(velMsg);

        tools::CbEffortArray effMsg;
        effMsg.effort = {can_boards.at(0).getEffortReal(),can_boards.at(1).getEffortReal(),can_boards.at(2).getEffortReal(),can_boards.at(3).getEffortReal(),can_boards.at(4).getEffortReal(),can_boards.at(5).getEffortReal()};
        effortPub.publish(effMsg);

        tools::CbPidArray positionPidMsgArray;
        tools::CbPid tmpPositionPidMsg;
        for (int j = 0; j < can_boards.size(); j++) {
            tmpPositionPidMsg.can_id = can_boards.at(j).getCanId();
            tmpPositionPidMsg.p = can_boards.at(j).getPositionPID().p;
            tmpPositionPidMsg.i = can_boards.at(j).getPositionPID().i;
            tmpPositionPidMsg.d = can_boards.at(j).getPositionPID().d;
            positionPidMsgArray.data.push_back(tmpPositionPidMsg);
        }
        positionPidsPublisher.publish(positionPidMsgArray);

        tools::CbPidArray velocityPidMsgArray;
        tools::CbPid tmpVelocityPidMsg;
        for (int j = 0; j < can_boards.size(); j++) {
            tmpVelocityPidMsg.can_id = can_boards.at(j).getCanId();
            tmpVelocityPidMsg.p = can_boards.at(j).getVelocityPID().p;
            tmpVelocityPidMsg.i = can_boards.at(j).getVelocityPID().i;
            tmpVelocityPidMsg.d = can_boards.at(j).getVelocityPID().d;
            velocityPidMsgArray.data.push_back(tmpVelocityPidMsg);
        }
        velocityPidsPublisher.publish(velocityPidMsgArray);

        tools::CbPositionLimitsArray posLimMsgArr;
        tools::CbPoseLimits tmpPosLimMsg;
        for (int i = 0; i < can_boards.size(); i++) {
            tmpPosLimMsg.can_id = can_boards.at(i).getCanId();
            tmpPosLimMsg.from = can_boards.at(i).getPositionLimits().from;
            tmpPosLimMsg.to = can_boards.at(i).getPositionLimits().to;
            posLimMsgArr.data.push_back(tmpPosLimMsg);
        }
        positionLimitsPub.publish(posLimMsgArr);

        tools::CbEffortLimitsArray effLimMsgArr;
        tools::CbEffortLimits tmpEffLimMsg;
        for (int i = 0; i < can_boards.size(); i++) {
            tmpEffLimMsg.can_id = can_boards.at(i).getCanId();
            tmpEffLimMsg.min = can_boards.at(i).getEffortLimits().min;
            tmpEffLimMsg.max = can_boards.at(i).getEffortLimits().max;
            effLimMsgArr.data.push_back(tmpEffLimMsg);
        }
        effortLimitsPub.publish(effLimMsgArr);

        tools::CbOffsetArray offsetMsg;
        offsetMsg.offset = {can_boards.at(0).getEncoderOffset(), can_boards.at(1).getEncoderOffset(), can_boards.at(2).getEncoderOffset(), can_boards.at(3).getEncoderOffset(), can_boards.at(4).getEncoderOffset(), can_boards.at(5).getEncoderOffset()};
        encoderOffsetPublisher.publish(offsetMsg);

        tools::CbFrequencyArray frequencyArrayMsg;
        frequencyArrayMsg.frequency = {can_boards.at(0).getEncoderReadingsFrequency(), can_boards.at(1).getEncoderReadingsFrequency(), can_boards.at(2).getEncoderReadingsFrequency(), can_boards.at(3).getEncoderReadingsFrequency(), can_boards.at(4).getEncoderReadingsFrequency(), can_boards.at(5).getEncoderReadingsFrequency()};
        readingsFrequencyPublisher.publish(frequencyArrayMsg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool CanBoards::setEncoderOffsetCallback(tools::encoder_set_offset::Request  &req, tools::encoder_set_offset::Response &res) {
    can_boards[req.id].setEncoderOffset(req.new_value);
    if (can_boards[req.id].getEncoderOffset() == req.new_value) {
        std::fstream file;
        file.open(globaloffsetsFilePath,std::ios_base::out);
        for(int i=0;i<can_boards.size();i++)
        {
            file << can_boards[i].getEncoderOffset() << std::endl;
        }
        file.close();
        res.success = true;
        return true;
    }
}

bool CanBoards::setEncoderPositionPidCallback(tools::cb_set_pid::Request  &req, tools::cb_set_pid::Response &res) {
    int s, i;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, canBoardsCanInterface);
    ioctl(s, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        exit(1);
    }
    if (s < 0) {
        std::cout << "Can socket error!" << std::endl;
    } else {
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &globalLoopback, sizeof(globalLoopback));
        can_frame frame;
        frame.can_id = can_boards.at(req.msg.can_id).getCanId();
        frame.can_dlc = 7; // Number of bytes of data to send
        frame.data[0] = 0x18; // Function type
        frame.data[1] = int((req.msg.p*1000)) >> 8; // first byte
        frame.data[2] = int((req.msg.p*1000)) & 0x00ff; // second byte
        frame.data[3] = int((req.msg.i*1000)) >> 8; // first byte
        frame.data[4] = int((req.msg.i*1000)) & 0x00ff; // second byte
        frame.data[5] = int((req.msg.d*1000)) >> 8; // first byte
        frame.data[6] = int((req.msg.d*1000)) & 0x00ff; // second byte
        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            //perror("Write");
        }
        if (close(s) < 0) {
            perror("Close");
        }
        res.success = true;
        return true;
    }
}

bool CanBoards::setEncoderVelocityPidCallback(tools::cb_set_pid::Request  &req, tools::cb_set_pid::Response &res) {
    int s, i;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, canBoardsCanInterface);
    ioctl(s, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        exit(1);
    }
    if (s < 0) {
        std::cout << "Can socket error!" << std::endl;
    } else {
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &globalLoopback, sizeof(globalLoopback));
        can_frame frame;
        frame.can_id = can_boards.at(req.msg.can_id).getCanId();
        frame.can_dlc = 7; // Number of bytes of data to send
        frame.data[0] = 0x19; // Function type
        frame.data[1] = int((req.msg.p*1000)) >> 8; // first byte
        frame.data[2] = int((req.msg.p*1000)) & 0x00ff; // second byte
        frame.data[3] = int((req.msg.i*1000)) >> 8; // first byte
        frame.data[4] = int((req.msg.i*1000)) & 0x00ff; // second byte
        frame.data[5] = int((req.msg.d*1000)) >> 8; // first byte
        frame.data[6] = int((req.msg.d*1000)) & 0x00ff; // second byte
        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            //perror("Write");
        }
        if (close(s) < 0) {
            perror("Close");
        }
        res.success = true;
        return true;
    }
}

bool CanBoards::setEncoderPositionLimitsCallback(tools::cb_set_pose_limits::Request  &req, tools::cb_set_pose_limits::Response &res) {
    int s, i;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, canBoardsCanInterface);
    ioctl(s, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        exit(1);
    }
    if (s < 0) {
        std::cout << "Can socket error!" << std::endl;
    } else {
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &globalLoopback, sizeof(globalLoopback));
        if ((req.msg.from >= -180) && (req.msg.from < 180) && (req.msg.to >= -180) && (req.msg.to < 180)) {
            can_frame frame;
            unsigned int tmp_from = positionToEncoderReadings(req.msg.from - 180.0); // what with offsets
            unsigned int tmp_to = positionToEncoderReadings(req.msg.to - 180.0);
            frame.can_id = can_boards.at(req.msg.can_id).getCanId();
            frame.can_dlc = 5; // Number of bytes of data to send
            frame.data[0] = 0x1A; // Function type
            frame.data[1] = tmp_from >> 8; // first byte;
            frame.data[2] = tmp_from & 0x00ff; // second byte;
            frame.data[3] = tmp_to >> 8; // first byte;
            frame.data[4] = tmp_to & 0x00ff; // second byte;
            if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
                //perror("Write");
            }
            if (close(s) < 0) {
                perror("Close");
            }
            res.success = true;
            return true;
        } else {
            res.success = false;
            return false;
        }
    }
}

bool CanBoards::setEncoderEffortLimitsCallback(tools::cb_set_effort_limits::Request  &req, tools::cb_set_effort_limits::Response &res) {
    int s, i;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, canBoardsCanInterface);
    ioctl(s, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        exit(1);
    }
    if (s < 0) {
        std::cout << "Can socket error!" << std::endl;
    } else {
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &globalLoopback, sizeof(globalLoopback));
        if((req.msg.min >= 0) && (req.msg.min <= 100) && (req.msg.max >= 0) && (req.msg.max <= 100) && (req.msg.min <= req.msg.max)) {
            can_frame frame;
            frame.can_id = can_boards.at(req.msg.can_id).getCanId();
            frame.can_dlc = 3; // Number of bytes of data to send
            frame.data[0] = 0x1B; // Function type
            frame.data[1] = req.msg.min; // first byte
            frame.data[2] = req.msg.max; // second byte
            if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
                //perror("Write");
            }
            if (close(s) < 0) {
                perror("Close");
            }
            res.success = true;
            return true;
        } else {
            res.success = false;
            return false;
        }
    }
}

bool CanBoards::setEncoderReadingsFrequencyCallback(tools::cb_set_frequency::Request  &req, tools::cb_set_frequency::Response &res) {
    int s, i;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, canBoardsCanInterface);
    ioctl(s, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        exit(1);
    }
    if (s < 0) {
        std::cout << "Can socket error!" << std::endl;
    } else {
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &globalLoopback, sizeof(globalLoopback));
        if ((req.frequency > 0) && (req.frequency <= 100)) {
            can_frame frame;
            frame.can_id = can_boards.at(req.can_id).getCanId();
            frame.can_dlc = 2; // Number of bytes of data to send
            frame.data[0] = 0x1C; // Function type
            frame.data[1] = 100/req.frequency; // Calculating divider
            if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
                //perror("Write");
            }
            if (close(s) < 0) {
                perror("Close");
            }
            res.success = true;
            return true;
        } else {
            res.success = false;
            return false;
        }
    }
}

void CanBoards::sendCanFrameRequest(int can_board_id, int can_frame_type) {
    int s, i;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, canBoardsCanInterface);
    ioctl(s, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        exit(1);
    }
    if (s < 0) {
        std::cout << "Can socket error!" << std::endl;
    } else {
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &globalLoopback, sizeof(globalLoopback));
        can_frame frame;
        frame.can_id = can_board_id;
        frame.can_dlc = 2; // Number of bytes of data to send
        frame.data[0] = 0x01; // Function type
        frame.data[1] = can_frame_type;
        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            //perror("Write");
        }
        if (close(s) < 0) {
            perror("Close");
        }
    }
}
