#!/usr/bin/env python

import rospy
import canopen
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from tools.srv import micontrols_state
from tools.srv import micontrols_switch
import threading
import time
import sys

reload(sys)
sys.setdefaultencoding('utf-8')

# CONFIG
WHEELS_TO_ID = {
    "fl": 1, "rl": 2, "rr": 3, "fr": 4
}

class Wheel:
    def __init__(self, wheel_id, network, micontrol_eds):
        self.id = wheel_id
        self.node = network.add_node(wheel_id, micontrol_eds)
        self.node.sdo['Device command']['Device command - execute on change'].raw = 0x15  # Mode SubVel
        print("Mode Sub Velocity")
        self._setup_pdo()
        self.node.sdo['SVel feedback'].raw = 0x094A
        self.node.sdo['Power enable'].raw = 0

    def set_speed(self, speed):
        global speed_const
        self.node.rpdo[4]['Device command.Device command - data 0'].raw = int(speed * speed_const)
        self.node.rpdo[4]['Device command.Device command - execute on change'].raw = 0x32

    def _setup_pdo(self):
        # Read current PDO configuration
        self.node.tpdo.read()
        self.node.rpdo.read()

        self.node.rpdo[4].clear()
        self.node.sdo['Device command']['Device command - data 1'].raw = 0
        self.node.rpdo[4].add_variable('Device command', 'Device command - data 0')
        self.node.rpdo[4].add_variable('Device command', 'Device command - execute on change')
        self.node.rpdo[4].enabled = True

        # Save new configuration (node must be in pre-operational)
        self.node.nmt.state = 'PRE-OPERATIONAL'
        self.node.rpdo.save()

        # Start RPDO4 with an interval of 100 ms
        self.node.rpdo[4]['Device command.Device command - data 0'].raw = 0
        self.node.rpdo[4]['Device command.Device command - execute on change'].raw = 0x32
        self.node.rpdo[4].start(0.01)
        print("PDO is set up")

class DriveModule:
    def __init__(self):
        # ROS Node setup
        rospy.init_node('wheel_controller', anonymous=True)

        # CANOpen setup
        self.network = canopen.Network()
        can_interface = rospy.get_param('~can_interface')
        micontrol_eds = rospy.get_param('~micontrol_eds')
        self.network.connect(channel=can_interface, bustype='socketcan')
        self.enabled = 0
        self.emergency_stop = False

        self.wheels = {
            wheel_name: Wheel(id_, self.network, micontrol_eds) for wheel_name, id_ in WHEELS_TO_ID.items()
        }

        self.wheel_sets = {
            "right_set": (self.wheels["fr"], self.wheels["rr"]),
            "left_set": (self.wheels["fl"], self.wheels["rl"]),
        }

        # ROS Node setup
        rospy.Subscriber('raw_cmd_vel', Twist, self.controller_data)

        print("Subscribed to controller")

        # Voltage monitor
        self.voltage_pub = rospy.Publisher('rover/voltage', Float64, queue_size=1)
        self.temp_pub = rospy.Publisher('rover/micontrols/temperature', Float32MultiArray, queue_size=1)
        self.current_pub = rospy.Publisher('rover/micontrols/motor_current', Float32MultiArray, queue_size=1)
        self.enable_pub = rospy.Publisher('rover/micontrols/enable', UInt16MultiArray, queue_size=1)

        self.ros_diagnostics_thread = threading.Thread(target=self.micontrol_monitor)
        self.ros_diagnostics_thread.start()

        # ROS Service to check micontrols state (on/off)
        rospy.Service('rover/get_micontrols_state', micontrols_state, self.handle_state_check)

        # ROS Service to enable/disable micontrols
        rospy.Service('rover/switch_micontrols', micontrols_switch, self.handle_switch)

    def controller_data(self, data):
        if self.emergency_stop:
            return

        if data.angular.z > 0:
            langular = 1 # [0;1]
            rangular = 1-abs(data.angular.z)*0.8 # [0.2;1]
        else:
            rangular = 1 # [0;1]
            langular = 1-abs(data.angular.z)*0.8 # [0.2;1]

        #rotation in place
        if data.linear.x == 0:
            self.wheels["fr"].set_speed(data.angular.z/4)
            self.wheels["rr"].set_speed(data.angular.z/4)
            self.wheels["fl"].set_speed(data.angular.z/4)
            self.wheels["rl"].set_speed(data.angular.z/4)
            return


        for wheel_set_key, wheel_set_value in self.wheel_sets.items():
            if wheel_set_key == "left_set":
                for wheel in wheel_set_value:
                    wheel.set_speed(langular * data.linear.x)
            if wheel_set_key == "right_set":
                for wheel in wheel_set_value:
                    wheel.set_speed(-rangular * data.linear.x)

    def handle_state_check(self, req):
        return self.enabled

    def handle_switch(self, req):
        if req.turnOn == 0:
            self.turn_off(self)
        else:
            self.turn_on(self)
        return True

    def turn_on(self, *args):
        rospy.loginfo("Enabling micontrols")
        self.network.nmt.state = 'OPERATIONAL'
        for wheel in self.wheels:
            self.wheels[wheel].node.sdo['Device command']['Device command - execute on change'].raw = 0x15  # Mode SubVel
            self.wheels[wheel].node.sdo['SVel feedback'].raw = 0x094A
            self.wheels[wheel].node.sdo['Power enable'].raw = 1
        self.enabled = 1

    def turn_off(self, *args):
        rospy.loginfo("Disabling micontrols")
        self.network.nmt.state = 'PRE-OPERATIONAL'
        for wheel in self.wheels:
            self.wheels[wheel].node.sdo['Power enable'].raw = 0
        self.enabled = 0


    def micontrol_monitor(self):
        while not rospy.is_shutdown():
            volt = 0.0

            temp = [0, 0, 0, 0]
            current = [0, 0, 0, 0]
            enable = [0, 0, 0, 0]

            try:
                for wheel in self.wheels:
                    volt += self.wheels[wheel].node.sdo["Electronic voltage - Ue"].raw  / 1000.0
                    temp[self.wheels[wheel].id - 1] = self.wheels[wheel].node.sdo["Temperature of power amplifier"].raw / 10.0
                    current[self.wheels[wheel].id - 1] = self.wheels[wheel].node.sdo["Motor current - Im"].raw / 1000.0
                    enable[self.wheels[wheel].id - 1] = self.wheels[wheel].node.sdo['Power enable'].raw
            except Exception as e:
                rospy.logerr(e)
                rospy.logwarn("Cannot read micontrol data")
                continue

            volt /= 4.0

            pub_temp = Float32MultiArray()
            pub_temp.data = temp

            current_temp = Float32MultiArray()
            current_temp.data = current

            enable_temp = UInt16MultiArray()
            enable_temp.data = enable

            self.temp_pub.publish(pub_temp)
            self.current_pub.publish(current_temp)
            self.voltage_pub.publish(volt)
            self.enable_pub.publish(enable_temp)

            # Emergency stop

            if volt < min_rover_voltage:
                self.emergency_stop = True

                for wheel in self.wheels:
                    self.wheels[wheel].set_speed(0)

                time.sleep(5)
                self.turn_off()

            time.sleep(1)

if __name__ == '__main__':
    global min_rover_voltage
    global speed_const

    min_rover_voltage = rospy.get_param('/rover/min_voltage', 21.0)
    speed_const = rospy.get_param('~speed_const', 201)

    drive = DriveModule()
    rospy.on_shutdown(drive.turn_off)
    rospy.spin()
    drive.network.disconnect()
