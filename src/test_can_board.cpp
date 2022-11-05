#include "../include/test_can_board.hpp"

CanBoard::CanBoard(int id)
{
    can_id = id;
    isEnabled = true;
    frameType = 0;
}

unsigned int positionToEncoderReadings(double position) {
    if (position > 180) {
        position = position - 360;
    } else if (position < -180) {
        position = position + 360;
    }
    int tmp_position = (position*4096)/360;
    while(tmp_position < 0)
        tmp_position += 4096;
    while(tmp_position >= 4096)
        tmp_position -= 4096;
    return tmp_position;
}

void CanBoard::workerCanSender()
{
    int s, i;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
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

    while (isEnabled) {
        if (s < 0) {
            std::cout << "Can socket error!" << std::endl;
        } else {
            switch(this->getFrameType()) {
                case 10: frame = this->getEffortFrame(); break;
                case 11: frame = this->getPositionFrame(); break;
                case 12: frame = this->getVelocityFrame(); break;
            }

            if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
                perror("Write");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }  
    }
    if (close(s) < 0) {
        perror("Close");
    }
}

void CanBoard::workerCanReceiver() {
    int s, i;
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

    while (isEnabled)
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

can_frame CanBoard::getEffortFrame() {
    if(getEffortGoal() < -100 || getEffortGoal() > 100)
        throw "Invalid effort value! Outside of range <-100, 100>";
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = 2; // Number of bytes of data to send
    frame.data[0] = 0x10; // Function type
    frame.data[1] = getEffortGoal(); // first byte
    return frame;
}

can_frame CanBoard::getPositionFrame() { 
     if(getPositionGoal() < -180 || getPositionGoal() > 180)
        throw "Invalid position value! Outside of range <-180, 180>";
    struct can_frame frame;
    unsigned int tmp_position = positionToEncoderReadings(getPositionGoal() + getEncoderOffset());
    frame.can_id = can_id;
    frame.can_dlc = 3; // Number of bytes of data to send
    frame.data[0] = 0x11; // Function type
    frame.data[1] = tmp_position >> 8; // first byte
    frame.data[2] = tmp_position & 0x00ff; // second byte
    return frame;
}

can_frame CanBoard::getVelocityFrame() {
    struct can_frame frame;
    int tmp_velocity = getVelocityGoal() * 1000;
    frame.can_id = can_id;
    frame.can_dlc = 3; // Number of bytes of data to send
    frame.data[0] = 0x12; // Function type
    frame.data[1] = tmp_velocity >> 8; // first byte
    frame.data[2] = tmp_velocity & 0x00ff; // second byte
    return frame;
}