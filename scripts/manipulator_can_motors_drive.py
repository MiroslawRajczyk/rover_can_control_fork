#!/usr/bin/env python

import can
import rospy
from tools.msg import MotorSignal

class ManipulatorCanMotorsDrive:
    def __init__(self):
        # Load parameters
        can_interface = rospy.get_param('~can_interface', "can0")
        # Create socket to CAN bus
        self.bus = can.interface.Bus(bustype='socketcan', channel=can_interface, bitrate=500000)
        # Topic with motors' speed to set
        rospy.Subscriber("/rover/control/manipulator/set_raw_motor_speed", MotorSignal, self.set_speed_callback)

    def set_speed_callback(self, data):
        motor_id_to_address = [0x10, 0x11, 0x12, 0x13, 0x14, 0x15] # Translates motor ID to motor address
        # Check if given ID is valid
        if(data.id < 0 or data.id > 5):
            return

        # Translate speed value from [-1, 1] to [0, 255]
        speed = abs(data.speed) * 255
        if speed > 255:
            speed = 255
        # Save direction code
        direction = 0x11 # default: stop code
        if data.speed < 0:
            direction = 0x00 # backward code
        if data.speed > 0:
            direction = 0x22 # forward code
        # Create frame
        frame = [0x01, int(direction), int(speed)]
        # Send frame
        msg = can.Message(arbitration_id=motor_id_to_address[data.id], data=frame, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError:
            rospy.logerr("CAN: Can't set motor speed")

if __name__ == "__main__":
    try:
        rospy.init_node('manipulator_can_motors_drive')
        drive = ManipulatorCanMotorsDrive()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
