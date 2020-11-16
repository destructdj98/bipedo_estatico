#!/usr/bin/env python
#!-*- using utf-8 -*-

from GPIO_CONFIG import *
from RPi import GPIO
from time import sleep
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import pi

class Encoder():
    def __init__(self):
        self.GPIO_CONFIG = GPIO_CONFIG()
        self.clk_motor = self.GPIO_CONFIG.clk_motor
        self.dt_motor = self.GPIO_CONFIG.dt_motor
        self.clkState_motor = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.dtState_motor = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.clkLastState_motor = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.counter_motor = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.pos = [0,0,0,0,0,0,0,0,0,0,0,0]

    def init_encoder(self):
        self.GPIO_CONFIG.mode_config()
        self.GPIO_CONFIG.set_gpio()
        self.firs_state_encoder()

    def firs_state_encoder(self):
        for i in range(0,12):
            self.clkLastState_motor[i] = GPIO.input(self.clk_motor[i])

    def reads_counts(self):
        self.cal_pos()
        for i in range(0,12):
            self.clkState_motor[i] = GPIO.input(self.clk_motor[i])
            self.dtState_motor[i] = GPIO.input(self.dt_motor[i])
            if self.clkState_motor[i] != self.clkLastState_motor[i]:
                if self.dtState_motor[i] != self.clkState_motor[i]:
                    self.counter_motor[i] += 1
                else:
                    self.counter_motor[i] -= 1

            self.clkLastState_motor[i] = self.clkState_motor[i]
            #sleep(0.01)

    def cal_pos(self):
        for i in range(0,12):
            self.pos[i] = (float(self.counter_motor[i]) * 2 * pi)/4741

    def counter(self):
        return self.pos

    def reset_encoder(self):
        self.clkState_motor = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.dtState_motor = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.clkLastState_motor = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.counter_motor = [0,0,0,0,0,0,0,0,0,0,0,0]

def Node_encoder():
    pub = rospy.Publisher("join_state", JointState, queue_size = 10)
    rospy.init_node("joint_state_publisher")
    rate = rospy.Rate(100)

    #configuracion GPIO
    encoder = Encoder()
    encoder.init_encoder()
    rospy.loginfo('GPIO_SET_MODE----BCM')
    rospy.loginfo('GPIO_SET_PIN')
    sleep(2)

    #self.pos_encoder = pos_encoder
    dato = JointState()
    header = Header()
    header.stamp = rospy.Time.now()
    dato.name = ["j1", "j2", "j3", "j4", "j5", "j6", "j7", "j8", "j9", "j10", "j11", "j12"]
    dato.position = [0,0,0,0,0,0,0,0,0,0,0,0]
    dato.velocity = []
    dato.effort = []

    while not rospy.is_shutdown():
        encoder.reads_counts()
        counts = encoder.counter_motor
        #self.dato.position.append(self.pos_encoder)
        #for i in range(0, 11):
        dato.position = [counts[0],0,0,0,0,0,0,0,0,0,0,0]
        rospy.loginfo(dato)    
        pub.publish(dato)
        rate.sleep()

if __name__ == "__main__":
    try:
        Node_encoder()

    except rospy.ROSInterruptException:
        pass
