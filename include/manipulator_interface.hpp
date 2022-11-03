#pragma once

#include <ros/ros.h>

#include "can_boards.hpp"

class ManipulatorInterface {
    private:

    public:
        void setEfforts(char efforts[6]);
        void setPositions(double positions[6]);
        void setVelocities(double velocities[6]);

        ManipulatorInterface(ros::NodeHandle &node_handle, CanBoards &can_boards);

};
