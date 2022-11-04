#include "../include/test_can_board.hpp"

CanBoard::CanBoard(int id)
{
    can_id = id;
    isEnabled = true;
}

void CanBoard::workerCanSender()
{
    int s, i;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    while (isEnabled)
    {
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        std::cout << "Can socket error!" << std::endl;
    }
    else
    {
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
    }
        //std::cout << "My can id: " << can_id << std::endl;
        frame.can_id = can_id;
        frame.can_dlc = 3; // Number of bytes of data to send
        frame.data[0] = 0x1B; // Function type
        frame.data[1] = 0x1B; // first byte
        frame.data[2] = this->getVelocityGoal(); // second byte
        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write");
        }

        if (close(s) < 0) {
            perror("Close");
        }
        //std::cout<< "Thread for can board id: "<<can_id<<" - message sent."<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void CanBoard::workerCanReceiver()
{
    int s, i;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct can_filter rfilter[1];

	rfilter[0].can_id   = can_id;
	rfilter[0].can_mask = 0xFFF;
	//rfilter[1].can_id   = 0x200;
	//rfilter[1].can_mask = 0x700;

    while (isEnabled)
    {
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        std::cout << "Can socket error!" << std::endl;
    }
    else
    {
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
    }
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

        nbytes = read(s, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            perror("Read");
        }

        printf("0x%03X [%d] ",frame.can_id, frame.can_dlc);

        for (i = 0; i < frame.can_dlc; i++)
            printf("%02X ",frame.data[i]);

        printf("\r\n");

        if (close(s) < 0) {
            perror("Close");
        }
    }
}