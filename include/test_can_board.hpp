#pragma once

#include <linux/can.h>
#include <libsocketcan.h>
#include <atomic>
#include <thread>
#include <iostream>
#include <chrono>

unsigned int positionToEncoderReadings(double position);

struct PID {
    std::atomic<double> p;
    std::atomic<double> i;
    std::atomic<double> d;
};

struct EffortLimits {
    std::atomic<char> min;
    std::atomic<char> max;
};

struct PositionLimits {
    std::atomic<double> from;
    std::atomic<double> to;
};

class CanBoard {
    private:
        //std::atomic<int> can_socket;
        //std::atomic<unsigned int> can_id{0};
        //std::atomic<unsigned int> board_type;
        ////std::atomic<double> encoder_offset{0.0);
        //std::atomic<char> set_effort;
        //std::atomic<char> measured_effort;
        ////std::atomic<double> set_position{0.0};
        //std::atomic<double> measured_position{0.0};
        //std::atomic<double> set_velocity{0.0};
        //std::atomic<double> measured_velocity{0.0};
        //std::atomic<double> encoder_frames_sending_frequency{0.0};
        //PID velocity_PID;
        //PID position_PID;
        //EffortLimits effort_limits;
        //PositionLimits position_limits;
        void sendFrameRequest(unsigned char requested_frame_ID);
    public:
        int can_id;
        CanBoard(int id);
        void worker();

};