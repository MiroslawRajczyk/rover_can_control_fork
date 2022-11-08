#!/usr/bin/env python

import rospy
import can
from tools.msg import StatusLED

class StatusLEDDriver:
    def __init__(self):
        # Default LED color and blinking status
        self.red = 0
        self.green = 0
        self.blue = 0
        self.blink = False
        # Node status parameters
        self.is_current_off = False # Status used for blinking: True - LED is OFF, False - LED is ON
        # CAN bus configuration
        can_interface = rospy.get_param('~can_interface', "can0")
        self.bus = can.interface.Bus(bustype='socketcan', channel=can_interface, bitrate=500000)
        # Subscribe to topick
        rospy.Subscriber("/rover/status_led", StatusLED, self.rover_status_callback)
        # Load previous LED state
        self.load_led_state()
        self.set_color_in_buffer()

    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rate.sleep()
            # If blinking is disabled don't do anything
            if not self.blink:
                continue
            # Blink LED
            if self.is_current_off:
                # Is OFF so turn it ON
                self.set_color_in_buffer()
                self.is_current_off = False
            else:
                # Is ON so turn it OFF
                self.disable()
                self.is_current_off = True

    def rover_status_callback(self, data):
        self.red = data.red
        self.green = data.green
        self.blue = data.blue
        self.blink = data.blink
        self.set_color_in_buffer()
        self.save_led_state()

    def set_color_in_buffer(self):
        # Create and send message
        command = [0x04, self.red, self.green, self.blue]
        msg = can.Message(arbitration_id=0x030, data=command, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError:
            rospy.logerr("CAN: status LED colos NOT sent")

    def disable(self):
        command = [0x04, 0, 0, 0]
        # Create and send message
        msg = can.Message(arbitration_id=0x030, data=command, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError:
            rospy.logerr("CAN: status LED colos NOT sent")

    def load_led_state(self):
        try:
            file= open('/home/miroslaw/status_led_state.txt', 'r')
            self.red = int(file.readline())
            self.green = int(file.readline())
            self.blue = int(file.readline())
            if int(file.readline()) == 1:
                self.blink = True
            else:
                self.blink = False
        except:
            pass

    def save_led_state(self):
        try:
            file= open('/home/miroslaw/status_led_state.txt', 'w')
            file.write(str(self.red) + "\n")
            file.write(str(self.green) + "\n")
            file.write(str(self.blue) + "\n")
            if self.blink:
                file.write(str(1) + "\n")
            else:
                file.write(str(0) + "\n")
        except:
            pass

if __name__ == "__main__":
    rospy.init_node('status_led_driver')
    SLEDD = StatusLEDDriver()
    SLEDD.spin()
