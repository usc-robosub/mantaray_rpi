#!/usr/bin/env python3

import time
import rospy
import busio
import math
import board
from adafruit_pcas9685 import PCA9685
from mantaray_rpi.msg import FloatStamped
import pandas as pd
import time

REVERSED_THRUSTERS = [0, 1, 2, 3, 4, 5]
VALID_VOLTAGES = [10, 12, 14, 16, 18, 20]

NUM_THRUSTERS = 8
MIN_THRUST = -1
MAX_THRUST = 1
MIN_ACCEL = 0.01
MAX_ACCEL = 0.1
DEFAULT_ACCEL = 0.05
UPDATE_DELAY = 20 # IN ms
STILL_UPDATING = 100 
PWM_FREQ = 218

running = True
updatingThrusters = True
PCA_ADDRESSES = [0x40, 0x41]
PCAs = [None, None]

thrusters = [None] * NUM_THRUSTERS
voltage = 19 # Was around 20 the last time I checked
min_voltage = 18
max_voltage = 20

'''
Thrusters mapping:
    Thruster 0: pca 0, channel 1
    Thruster 1: pca 0, channel 0
    Thruster 2: pca 0, channel 3
    Thruster 3: pca 0, channel 2
    Thruster 4: pca 1, channel 3
    Thruster 5: pca 1, channel 0
    Thruster 6: pca 1, channel 5
    Thruster 7: pca 1, channel 7
'''

def microseconds_to_int16(time, freq):
    return time*freq*65536/1000000

class Thruster: # Agnostic to the direction of thrust. Will need to keep track of its own thruster num
    def __init__(self, thruster_num, channel, pca_num):
        self.thruster_num = thruster_num
        self.channel_num = channel
        self.pca_num = pca_num
        self.accel = DEFAULT_ACCEL
        self.PCA9685 = None
        self.targetThrust = 0
        self.currentThrust = 0
        self.stopping = False
        self.running = True
        self.initialized = False

        self.data_file = "20 V/force_pwm.csv"
        self.iodata = pd.read_csv(self.data_file)

    def setTargetThrust(self, target):
        global updatingThrusters
        if (target < MIN_THRUST):
            target = MIN_THRUST
        elif (target >MAX_THRUST):
            target = MAX_THRUST
        if (target == self.targetThrust):
            return
        else: # MIN accel doesn't apply to here
            if (self.targetThrust-target > MAX_ACCEL):
                target = self.targetThrust-MAX_ACCEL
            elif (target-self.targetThrust > MAX_ACCEL):
                target = MAX_ACCEL-self.targetThrust
            self.targetThrust = target
        updatingThrusters = True
    
    def setAccel(self, accel):
        if (accel < MIN_ACCEL):
            accel = MIN_ACCEL
        elif (accel >MAX_ACCEL):
            accel = MAX_ACCEL
        if (accel == self.accel):
            return
        else:
            self.accel = accel

    def setPca(self, pca_num):
        self.pca_num = pca_num

    def setPcaChannel(self, channel):
        self.channel_num = channel

    def stopThrusters(self):
        self.stopping = True
            
    def update_pwm(self):
            global voltage, VALID_VOLTAGES
            # This calculates the approximate PWM signal necessary to get the needed thrust
            if self.thruster_num in REVERSED_THRUSTERS:
                thrust = -self.currentThrust
            else:
                thrust = self.currentThrust
            df_closest = self.iodata.iloc[(self.iodata[' Force (Kg f)']-thrust).abs().argsort()[:1]]
            # df_closest = self.iodata.iloc[(self.iodata[' Force (Kg f)']-self.currentThrust).abs().argsort()[:2]] # returns the 2 closest pwm values
            self.pwm = df_closest.values[0][2]

    def thrusterCallback(self, data):
        # A callback for the mantaray/thruster_{num} topic
        self.setTargetThrust(data.data)

    def update(self):
        global updatingThrusters
        if self.running:
            # print("is running")
            # print("current thrust: " + str(self.currentThrust))
            # print("current taget: " + str(self.targetThrust))
            if self.stopping:
                print("Stopping")
                self.targetThrust = 0
                if self.currentThrust == self.targetThrust:
                    self.running = False
                    return
            if self.targetThrust != self.currentThrust:
                if (self.targetThrust > self.currentThrust):
                    # print("previous self.currentThrust of thruster " + str(self.thruster_num) +": " + str(self.currentThrust))
                    self.currentThrust+=min(self.targetThrust-self.currentThrust, self.accel)
                    # print("New self.currentThrust of thruster " + str(self.thruster_num) +": " + str(self.currentThrust))
                elif (self.targetThrust < self.currentThrust):
                    # print("previous self.currentThrust of thruster " + str(self.thruster_num) +": " + str(self.currentThrust))
                    self.currentThrust-=min(self.currentThrust-self.targetThrust, self.accel)
                    # print("New self.currentThrust of thruster " + str(self.thruster_num) +": " + str(self.currentThrust))
                updatingThrusters=True
            else:
                return
            # print("Thruster"+str(self.thruster_num)+" pca"+str(self.pca_num)+" channel"+str(self.channel_num)+" freq"+str(PWM_FREQ)+" currentThrust"+str(self.currentThrust))
            self.update_pwm()
            PCAs[self.pca_num].channels[self.channel_num].duty_cycle = round(microseconds_to_int16(self.pwm, PCAs[self.pca_num].frequency))            
        elif not self.initialized:
            PCAs[self.pca_num].channels[self.channel_num].duty_cycle = round(microseconds_to_int16(self.pwm, PCAs[self.pca_num].frequency))            
            self.initialized=True

def initPcas(addresses = [0x40], freq = PWM_FREQ, debug = False):
    global PCAs
    i2c_bus = busio.I2C(board.SCL, board.SDA)
    for i in range(len(addresses)):
        PCAs[i] = PCA9685(i2c_bus, address=addresses[i])
        PCAs[i].frequency = freq
    if (debug):
        print("pcas have been initialized")

def initThrusters(debug = False):
    '''
    Thrusters mapping:
    Thruster 0: pca 0, channel 2
    Thruster 1: pca 0, channel 1
    Thruster 2: pca 0, channel 4
    Thruster 3: pca 0, channel 3
    Thruster 4: pca 1, channel 3
    Thruster 5: pca 1, channel 1
    Thruster 6: pca 1, channel 2
    Thruster 7: pca 1, channel 4
    '''
    global thrusters
    thrusters[0] = Thruster(0, 2, 0)
    thrusters[1] = Thruster(1, 1, 0)
    thrusters[2] = Thruster(2, 4, 0)
    thrusters[3] = Thruster(3, 3, 0)
    thrusters[4] = Thruster(4, 3, 1)
    thrusters[5] = Thruster(5, 1, 1)
    thrusters[6] = Thruster(6, 2, 1)
    thrusters[7] = Thruster(7, 4, 1)

    for i in range(NUM_THRUSTERS):
        rospy.Subscriber("/mantaray/thruster"+ str(i) + "/input", FloatStamped, thrusters[0].thrustCallback)
    for i in range(NUM_THRUSTERS):
        init_thrusts = [k/1000 for k in range(-100, 100, 20)]
        stopping_thrusts = [k/1000 for k in range(100, 0, -20)]
        for j in init_thrusts:
            thrusters[i].setTargetThrust(j)
            thrusters[i].update()
            time.sleep(0.02)
        time.sleep(0.5)
        for j in stopping_thrusts:
            thrusters[i].setTargetThrust(j)
            thrusters[i].update()
            time.sleep(0.02)
        if debug:
            print("Thruster " + str(i) + " has been initialized")
    if debug:
        print("Thrusters have been initialized")

# def simple_thrusters_test():
#     initPcas(addresses=[0x40, 0x41],debug = True)
#     # print()
#     initThrusters(debug = True)
#     init_thrusts = [k/1000 for k in range(-1000, 1000, 20)]
#     stopping_thrusts = [k/1000 for k in range(1000, 0, -20)]
#     for i in range(NUM_THRUSTERS):
#         for j in init_thrusts:
#             thrusters[i].setTargetThrust(j)
#             thrusters[i].update()
#             time.sleep(0.02)
#         time.sleep(0.5)
#         for j in stopping_thrusts:
#             thrusters[i].setTargetThrust(j)
#             thrusters[i].update()
#             time.sleep(0.02)
#         time.sleep(5)

if __name__ == "__main__":
    initPcas(addresses=[0x40, 0x41],debug = True)
    initThrusters(debug = True)
    rospy.spin() # can add a whlie loop for doing stuff later