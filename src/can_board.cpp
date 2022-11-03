#include "../include/can_board.hpp"

CanBoard::CanBoard(int id) {
    this->can_id = id;
}

unsigned int positionToEncoderReadings(double position) {
    int tmp_position = (position*4096)/360;
    while(tmp_position < 0)
        tmp_position += 4096;
    while(tmp_position >= 4096)
        tmp_position -= 4096;
    return tmp_position;
}

void CanBoard::sendFrameRequest(unsigned char requested_frame_ID) {
    can_frame frame;
    frame.can_id = this->can_id;
    frame.can_dlc = 2; // Number of bytes of data to send
    frame.data[0] = 0x01; // Function type
    frame.data[1] = requested_frame_ID;
    if (write(this->can_socket, &frame, sizeof(can_frame)) != sizeof(can_frame))
        throw "Error occurred while writing to CAN bus";
}

void CanBoard::setEffort(char effort) {
    if(effort < -100 || effort > 100)
        throw "Invalid effort value! Outside of range <-100, 100>";
    can_frame frame;
    frame.can_id = this->can_id;
    frame.can_dlc = 2; // Number of bytes of data to send
    frame.data[0] = 0x10; // Function type
    frame.data[1] = effort;
    if (write(this->can_socket, &frame, sizeof(can_frame)) != sizeof(can_frame))
        throw "Error occurred while writing to CAN bus";
}

void CanBoard::requestSetEffort() {
    sendFrameRequest(0x10);
}

char CanBoard::getSetEffort() {
    return this->set_effort;
}

char CanBoard::getMeasuredEffort() {
    return this->measured_effort;
}

void CanBoard::setPosition(double position) {
    if(position < -180 || position > 180)
        throw "Invalid position value! Outside of range <-180, 180>";
    can_frame frame;
    unsigned int tmp_position = positionToEncoderReadings(position + encoder_offset);
    frame.can_id = this->can_id;
    frame.can_dlc = 3; // Number of bytes of data to send
    frame.data[0] = 0x11; // Function type
    frame.data[1] = tmp_position >> 8; // first byte
    frame.data[2] = tmp_position & 0x00ff; // second byte
    if (write(this->can_socket, &frame, sizeof(can_frame)) != sizeof(can_frame))
        throw "Error occurred while writing to CAN bus";
}

void CanBoard::requestSetPosition() {
    sendFrameRequest(0x11);
}

double CanBoard::getSetPosition() {
    return this->set_position;
}

double CanBoard::getMeasuredPosition() {
    return this->measured_position;
}

void CanBoard::setVelocity(double velocity) {
    can_frame frame;
    int tmp_velocity = velocity * 1000;
    frame.can_id = this->can_id;
    frame.can_dlc = 3; // Number of bytes of data to send
    frame.data[0] = 0x12; // Function type
    frame.data[1] = tmp_velocity >> 8; // first byte
    frame.data[2] = tmp_velocity & 0x00ff; // second byte
    if (write(this->can_socket, &frame, sizeof(can_frame)) != sizeof(can_frame))
        throw "Error occurred while writing to CAN bus";
}

void CanBoard::requestSetVelocity() {
    sendFrameRequest(0x12);
}

double CanBoard::getSetVelocity() {
    return this->set_velocity;
}

double CanBoard::getMeasuredVelocity() {
    return this->measured_velocity;
}

void CanBoard::setLEDColor(ColorRGB color) {
    can_frame frame;
    frame.can_id = this->can_id;
    frame.can_dlc = 4; // Number of bytes of data to send
    frame.data[0] = 0x20; // Function type
    frame.data[1] = color.red;
    frame.data[2] = color.green;
    frame.data[3] = color.blue;
    if (write(this->can_socket, &frame, sizeof(can_frame)) != sizeof(can_frame))
        throw "Error occurred while writing to CAN bus";
}

void CanBoard::requestSetLEDColor() {
    sendFrameRequest(0x20);
}

ColorRGB CanBoard::getLEDColor() {
    return this->led_color; // multithreaded safety
}

void CanBoard::setEncoderFramesSendingFrequency(double frequency) {
    if(frequency < 0 || frequency > 100)
        throw "Invalid frequency value! Outside of range <0, 100>";
    can_frame frame;
    frame.can_id = this->can_id;
    frame.can_dlc = 2; // Number of bytes of data to send
    frame.data[0] = 0x1C; // Function type
    frame.data[1] = 100/frequency; // Calculating divider
    if (write(this->can_socket, &frame, sizeof(can_frame)) != sizeof(can_frame))
        throw "Error occurred while writing to CAN bus";
}

void CanBoard::requestSetEncoderFramesSendingFrequency() {
    sendFrameRequest(0x1C);
}

double CanBoard::getEncoderFramesSendingFrequency() {
    return this->encoder_frames_sending_frequency;
}

void CanBoard::setVelocityPID(PID pid) {
    can_frame frame;
    frame.can_id = this->can_id;
    frame.can_dlc = 7; // Number of bytes of data to send
    frame.data[0] = 0x19; // Function type
    frame.data[1] = (pid.p*1000) >> 8; // first byte
    frame.data[2] = (pid.p*1000) & 0x00ff; // second byte
    frame.data[3] = (pid.i*1000) >> 8; // first byte
    frame.data[4] = (pid.i*1000) & 0x00ff; // second byte
    frame.data[5] = (pid.d*1000) >> 8; // first byte
    frame.data[6] = (pid.d*1000) & 0x00ff; // second byte
    if (write(this->can_socket, &frame, sizeof(can_frame)) != sizeof(can_frame))
        throw "Error occurred while writing to CAN bus";
}

void CanBoard::requestSetVelocityPID() {
    sendFrameRequest(0x19);
}

PID CanBoard::getVelocityPID() {
    return this->velocity_PID; // multithreaded safety
}

void CanBoard::setPositionPID(PID pid) {
    can_frame frame;
    frame.can_id = this->can_id;
    frame.can_dlc = 7; // Number of bytes of data to send
    frame.data[0] = 0x18; // Function type
    frame.data[1] = (pid.p*1000) >> 8; // first byte
    frame.data[2] = (pid.p*1000) & 0x00ff; // second byte
    frame.data[3] = (pid.i*1000) >> 8; // first byte
    frame.data[4] = (pid.i*1000) & 0x00ff; // second byte
    frame.data[5] = (pid.d*1000) >> 8; // first byte
    frame.data[6] = (pid.d*1000) & 0x00ff; // second byte
    if (write(this->can_socket, &frame, sizeof(can_frame)) != sizeof(can_frame))
        throw "Error occurred while writing to CAN bus";
}

void CanBoard::requestSetPositionPID() {
    sendFrameRequest(0x18);
}

PID CanBoard::getPositionPID() {
    return this->position_PID; // multithreaded safety
}

void CanBoard::setEffortLimits(EffortLimits effort_limits) {
    can_frame frame;
    frame.can_id = this->can_id;
    frame.can_dlc = 3; // Number of bytes of data to send
    frame.data[0] = 0x1B; // Function type
    frame.data[1] = effort_limits.min; // first byte
    frame.data[2] = effort_limits.max; // second byte
    if (write(this->can_socket, &frame, sizeof(can_frame)) != sizeof(can_frame))
        throw "Error occurred while writing to CAN bus";
}

void CanBoard::requestSetEffortLimits() {
    sendFrameRequest(0x1B);
}

EffortLimits CanBoard::getEffortLimits() {
    return this->effort_limits; // multithreaded safety
}

void CanBoard::setPositionLimits(PositionLimits position_limits) {
    can_frame frame;
    unsigned int tmp_from = positionToEncoderReadings(position_limits.from); // what with offsets
    unsigned int tmp_to = positionToEncoderReadings(position_limits.to);
    frame.can_id = this->can_id;
    frame.can_dlc = 7; // Number of bytes of data to send
    frame.data[0] = 0x18; // Function type
    frame.data[1] = tmp_from >> 8; // first byte;
    frame.data[2] = tmp_from & 0x00ff; // second byte;
    frame.data[3] = tmp_to >> 8; // first byte;
    frame.data[4] = tmp_to & 0x00ff; // second byte;
    if (write(this->can_socket, &frame, sizeof(can_frame)) != sizeof(can_frame))
        throw "Error occurred while writing to CAN bus";
}

void CanBoard::requestSetPositionLimits() {
    sendFrameRequest(0x18); 
}

PositionLimits CanBoard::getPositionLimits() {
    return this->position_limits; // multithreaded safety
}

bool CanBoard::canCallback(char data[], unsigned int data_length) {
    if(data_length < 2)
        return false;
    switch (data[0])
    {
    case 0x00: // Heartbeats
        if(data_length != 2)
            return false;
        this->board_type = data[1];
        break;
    
    case 0x10: // Effort setting
        if(data_length != 2)
            return false;
        this->set_effort = data[1];
        this->set_position = 0;
        this->set_velocity = 0;
        break;

    case 0x11: // Position setting
        if(data_length != 3)
            return false;
        this->set_effort = 0;
        this->set_position = ((data[1] << 8) | data[2]); // from encoder to our position
        this->set_velocity = 0;
        break;

    case 0x12: // Velocity setting
        if(data_length != 3)
            return false;
        this->set_effort = 0;
        this->set_position = 0;
        this->set_velocity = (data[1] << 8) | data[2];
        break;

    case 0x13: // Encoder position, velocity and effort update
        if(data_length != 6)
            return false;
        this->measured_position = (data[1] << 8) | data[2]; // from encoder to our position
        this->measured_velocity = (data[3] << 8) | data[4];
        this->measured_effort = data[5];
        break;

    case 0x18: // Position PID setting
        if(data_length != 7)
            return false;
        PID tmp_pid;
        tmp_pid.p = (data[1] << 8) | data[2];
        tmp_pid.i = (data[3] << 8) | data[4];
        tmp_pid.d = (data[5] << 8) | data[6];
        this->position_PID = tmp_pid; // multithreaded safety
        break;

    case 0x19: // Velocity PID setting
        if(data_length != 7)
            return false;
        PID tmp_pid;
        tmp_pid.p = (data[1] << 8) | data[2];
        tmp_pid.i = (data[3] << 8) | data[4];
        tmp_pid.d = (data[5] << 8) | data[6];
        this->velocity_PID = tmp_pid; // multithreaded safety
        break;

    case 0x1A: // Position limits setting
        if(data_length != 5)
            return false;
        PositionLimits tmp_position_limits;
        tmp_position_limits.from = (data[1] << 8) | data[2];
        tmp_position_limits.to = (data[3] << 8) | data[4];
        this->position_limits = tmp_position_limits; // multithreaded safety
        break;

    case 0x1B: // Effort limits setting
        if(data_length != 3)
            return false;
        EffortLimits tmp_effort_limits;
        tmp_effort_limits.min = data[1];
        tmp_effort_limits.max = data[2];
        this->effort_limits = tmp_effort_limits; // multithreaded safety
        break;

    case 0x1C: // Encoder frames sending frequency setting
        if(data_length != 2)
            return false;
        this->encoder_frames_sending_frequency = 100/data[1];
        break;

    case 0x20: // RGB LED color setting
        if(data_length != 4)
            return false;
        ColorRGB tmp_rgb_color;
        tmp_rgb_color.red = data[1];
        tmp_rgb_color.green = data[2];
        tmp_rgb_color.blue = data[3];
        this->led_color = tmp_rgb_color; // multithreaded safety
        break;

    default:
        return false;
    }
    return true;
}