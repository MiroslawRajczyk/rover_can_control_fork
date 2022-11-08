#!/usr/bin/env python

import can
import rospy
from tools.msg import Encoders
from tools.srv import encoder_set_offset
from tools.srv import encoder_get_offset

class ManipulatorEncodersDriver:
    def __init__(self):
        # Load parameters
        filters = [{"can_id" : 0x10, "can_mask" : 0x7F0, "extended" : False}]
        can_interface = rospy.get_param('~can_interface', "can0")
        self.update_rate = rospy.get_param('~update_rate', 1)
        # Create socket to CAN bus
        self.bus = can.interface.Bus(bustype='socketcan', channel=can_interface, can_filters=filters)
        # Topic to publish encoders values
        self.encoders_values_pub = rospy.Publisher('/rover/control/manipulator/encoders_values', Encoders, queue_size=1)
        # Service to set offset value
        rospy.Service('set_offset', encoder_set_offset, self.handle_set_offset)
        # Service to get offset value
        rospy.Service('get_offset', encoder_get_offset, self.handle_get_offset)
        # Parameters
        self.encoder_value = [0,0,0,0,0,0]
        self.encoder_offset = [0,0,0,0,0,0]
        try:
            self.load_offsets()
        except:
            pass

    def spin(self):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            try:
                can_msg = self.bus.recv()
                recv_msg = list(can_msg.data)
                id = recv_msg[0] - 16
                # Check if device id is valid
                if id < 0 or id > 5:
                    continue
                # Check if task code is "encoder value"
                if recv_msg[1] != 0:
                    continue
                self.encoder_value[id] = self.encoder_offset[id] + 360.0*(256 * recv_msg[2] + recv_msg[3])/4096.0
                self.encoder_value[id] -= 360 * (int(self.encoder_value[id]) / 360)
                msg = Encoders()
                msg.position = self.encoder_value
                self.encoders_values_pub.publish(msg)
            except:
                continue
            #rate.sleep()

    def handle_set_offset(self, req):
        self.encoder_offset[req.id] = req.new_value
        self.save_offsets()
        return True

    def handle_get_offset(self, req):
        return self.encoder_offset[req.id]

    def load_offsets(self):
        file= open('/home/miroslaw/manipluator_encoders_offests.txt', 'r')
        current_offset_number = 0
        for line in file:
            self.encoder_offset[current_offset_number] = float(line)
            current_offset_number += 1

    def save_offsets(self):
        file= open('/home/miroslaw/manipluator_encoders_offests.txt', 'w')
        for offset in self.encoder_offset:
            file.write(str(offset) + "\n")

if __name__ == "__main__":
    try:
        rospy.init_node('manipulator_encoders_driver')
        driver = ManipulatorEncodersDriver()
        driver.spin()
    except rospy.ROSInterruptException:
        pass
