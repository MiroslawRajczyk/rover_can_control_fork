#pragma once

#include <linux/can.h>
#include <atomic>

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

struct ColorRGB {
    std::atomic<unsigned char> red;
    std::atomic<unsigned char> green;
    std::atomic<unsigned char> blue;
};

class CanBoard {
    private:
        std::atomic<int> can_socket;
        std::atomic<unsigned int> can_id;
        std::atomic<unsigned int> board_type;
        std::atomic<double> encoder_offset;
        std::atomic<char> set_effort;
        std::atomic<char> measured_effort;
        std::atomic<double> set_position;
        std::atomic<double> measured_position;
        std::atomic<double> set_velocity;
        std::atomic<double> measured_velocity;
        ColorRGB led_color;
        std::atomic<double> encoder_frames_sending_frequency;
        PID velocity_PID;
        PID position_PID;
        EffortLimits effort_limits;
        PositionLimits position_limits;
        void sendFrameRequest(unsigned char requested_frame_ID);
    public:
        CanBoard(int id);
        void setEffort(char effort); V
        void requestSetEffort();
        char getSetEffort(); V
        char getMeasuredEffort(); V
        void setPosition(double position); V
        void requestSetPosition();
        double getSetPosition(); V
        double getMeasuredPosition(); V
        void setVelocity(double velocity); V
        void requestSetVelocity(); V
        double getSetVelocity(); V
        double getMeasuredVelocity(); V
        void setLEDColor(ColorRGB color);
        void requestSetLEDColor(); !
        ColorRGB getLEDColor();
        void setEncoderFramesSendingFrequency(double frequency); !
        void requestSetEncoderFramesSendingFrequency(); !
        double getEncoderFramesSendingFrequency(); !
        void setVelocityPID(PID pid); !
        void requestSetVelocityPID(); !
        PID getVelocityPID(); !
        void setPositionPID(PID pid); !
        void requestSetPositionPID(); !
        PID getPositionPID(); !
        void setEffortLimits(EffortLimits effort_limits); !
        void requestSetEffortLimits(); !
        EffortLimits getEffortLimits(); !
        void setPositionLimits(PositionLimits position_limits); !
        void requestSetPositionLimits(); ! 
        PositionLimits getPositionLimits(); !
        bool canCallback(char data[], unsigned int data_length); // return true - correct data frame; return false - incorrect data frame
};