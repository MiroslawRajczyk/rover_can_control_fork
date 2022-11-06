#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <atomic>
#include <thread>
#include <iostream>
#include <chrono>

unsigned int positionToEncoderReadings(double position);

struct PID {
    float p;
    float i;
    float d;
};

struct EffortLimits {
    int min;
    int max;
};

struct PositionLimits {
    float from;
    float to;
};

class CanBoard {
    private:
        int can_id;
        int boardType;
        bool isEnabled;
        int frameType;
        float velocityGoal;
        float velocityReal;
        float positionGoal;
        float positionReal;
        int effortGoal;
        int effortReal;
        float encoderOffset;
        float encoderReadingsFrequency;
        PID velocityPID;
        PID positionPID;
        EffortLimits effortLimits;
        PositionLimits positionLimits;
        //void sendFrameRequest(unsigned char requested_frame_ID);
    public:
        CanBoard(int id);
        void workerCanSender();
        can_frame getEffortFrame();
        can_frame getPositionFrame();
        can_frame getVelocityFrame();

        // getters & setters
        int getCanId() { return can_id; };
        void setCanId(int id) { can_id = id; };
        int getBoardType() { return boardType; };
        void setBoardType(int type) { boardType = type; };
        bool getIsEnabled() { return isEnabled; };
        void setIsEnabled(bool state) { isEnabled = state; };
        int getFrameType() { return frameType; };
        void setFrameType(int type) { frameType = type; };
        float getVelocityGoal() { return velocityGoal; };
        void setVelocityGoal(float velocity) { velocityGoal = velocity; };
        float getVelocityReal() { return velocityReal; };
        void setVelocityReal(float velocity) { velocityReal = velocity; };
        float getPositionGoal() { return positionGoal; };
        void setPositionGoal(float position) { positionGoal = position; };
        float getPositionReal() { return positionReal; };
        void setPositionReal(float position) { positionReal = position; };
        int getEffortGoal() { return effortGoal; };
        void setEffortGoal(int effort) { effortGoal = effort; };
        int getEffortReal() { return effortReal; };
        void setEffortReal(int effort) { effortReal = effort; };
        float getEncoderOffset() { return encoderOffset; };
        void setEncoderOffset(float offset) { encoderOffset = offset; };
        float getEncoderReadingsFrequency() { return encoderReadingsFrequency; };
        void setEncoderReadingsFrequency(float frequency) { encoderReadingsFrequency = frequency; };
        const PID & getVelocityPID() const { return velocityPID; }
        void setVelocityPID(float p, float i, float d) { velocityPID.p = p; velocityPID.i = i; velocityPID.d = d; };
        const PID & getPositionPID() const { return positionPID; }
        void setPositionPID(float p, float i, float d) { positionPID.p = p; positionPID.i = i; positionPID.d = d; };
        const EffortLimits & getEffortLimits() const { return effortLimits; }
        void setEffortLimits(int min, int max) { effortLimits.min = min; effortLimits.max = max; };
        const PositionLimits & getPositionLimits() const { return positionLimits; }
        void setPositionLimits(int from, int to) { positionLimits.from = from; positionLimits.to = to; };


};