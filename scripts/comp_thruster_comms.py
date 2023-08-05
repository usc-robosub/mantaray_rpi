#!/usr/bin/env python3   

import time
import rospy
import pandas as pd
import rospkg
import math
# import busio
# import board
# from adafruit_pcas9685 import PCA9685
# import pandas as pd
# from mantaray_rpi.msg import FloatStamped
import time
from std_msgs.msg import Float64
REVERSED_THRUSTERS = [2, 3, 5, 6]
VALID_VOLTAGES = [10, 12, 14, 16, 18, 20]

NUM_THRUSTERS = 8

# For real
MIN_THRUST = -40
MAX_THRUST = 40
MIN_ACCEL = 0.2
MAX_ACCEL = 1
DEFAULT_ACCEL = 0.5
UPDATE_DELAY = 20 # IN ms
PWM_FREQ = 218

SIM_MULT = 30
running = True
updatingThrusters = True
PCA_ADDRESSES = [0x40, 0x41]
PCAs = [None, None]
pubs = [None] * NUM_THRUSTERS

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
    return round(time*freq*65536/1000000)

class Thruster: # Agnostic to the direction of thrust. Will need to keep track of its own thruster num
    def __init__(self, thruster_num, channel, pca_num, output_type="real", limitAccel=True):
        self.thruster_num = thruster_num
        self.channel_num = channel
        self.pca_num = pca_num
        self.accel = DEFAULT_ACCEL
        self.targetThrust = 0
        self.currentThrust = 0
        self.stopping = False
        self.running = True
        self.initialized = False
        self.output_type = output_type
        self.limitAccel=limitAccel

        self.thrustMult = SIM_MULT if self.output_type == "simulation" else 1

        r = rospkg.RosPack()
        dir = r.get_path('mantaray_rpi')
        self.data_file = dir + "/data/20 V/force_pwm.csv"   
        self.iodata = pd.read_csv(self.data_file)

    def setTargetThrust(self, target):
        global updatingThrusters
        if (target < MIN_THRUST):
            target = MIN_THRUST
        elif (target >MAX_THRUST):
            target = MAX_THRUST
        if (target == self.targetThrust):
            return
        else:
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

    def thrusterCallback(self, data):
        # A callback for the mantaray/thruster_{num} topic
        # if (self.currentThrust != data.data):
        #     print("Thruster["+ str(self.thruster_num) + "]: currentThrust:"+str(self.currentThrust))
        #     print("data.data:" + str(data.data))
        #     print("targetThrust:" + str(self.targetThrust))
        # if (self.currentThrust != data.data):
        #     print(self.thruster_num)
        self.setTargetThrust(data.data)
        

    def get_duty_cycle(self):
        # Gets a normalized number from [-127, 127] to [1100, 1900]
        if self.thruster_num in REVERSED_THRUSTERS:
            return microseconds_to_int16(1530 + (-self.currentThrust * 3.15), PCAs[self.pca_num].frequency) 
        return microseconds_to_int16(1530 + (self.currentThrust * 3.15), PCAs[self.pca_num].frequency)

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
                    if (self.limitAccel):
                        self.currentThrust+=min(self.targetThrust-self.currentThrust, self.accel)
                    else:
                        # rospy.logdebug("Didn't limit accel")
                        self.currentThrust = self.targetThrust
                    # print("New self.currentThrust of thruster " + str(self.thruster_num) +": " + str(self.currentThrust))
                elif (self.targetThrust < self.currentThrust):
                    # print("previous self.currentThrust of thruster " + str(self.thruster_num) +": " + str(self.currentThrust))
                    if (self.limitAccel):
                        self.currentThrust-=min(self.currentThrust-self.targetThrust, self.accel)
                    else:
                        # rospy.logdebug("Didn't limit accel")
                        self.currentThrust = self.targetThrust
                    # print("New self.currentThrust of thruster " + str(self.thruster_num) +": " + str(self.currentThrust))
                updatingThrusters=True
            else:
                return
            # print("Thruster"+str(self.thruster_num)+" pca"+str(self.pca_num)+" channel"+str(self.channel_num)+" freq"+str(PWM_FREQ)+" currentThrust"+str(self.currentThrust))
            if self.output_type == "real":
                PCAs[self.pca_num].channels[self.channel_num].duty_cycle = self.get_duty_cycle()       
            elif self.output_type == "simulation":
                msg = FloatStamped()
                msg.header.stamp = rospy.Time.now()
                msg.data = self.currentThrust * self.thrustMult
                pubs[self.thruster_num].publish(msg)
        elif not self.initialized:
            PCAs[self.pca_num].channels[self.channel_num].duty_cycle = self.get_duty_cycle()     
            self.initialized=True
 
def initPcas(addresses = [0x40], freq = PWM_FREQ, debug = False):
    global PCAs
    i2c_bus = busio.I2C(board.SCL, board.SDA)
    for i in range(len(addresses)):
        PCAs[i] = PCA9685(i2c_bus, address=addresses[i])
        PCAs[i].frequency = freq
    if (debug):
        print("pcas have been initialized")

def initPubs(debug = False):
    global pubs
    for i in range(len(pubs)):
        pubs[i] = rospy.Publisher("/mantaray/thrusters/"+str(i)+"/input", FloatStamped, queue_size = 10)
    if (debug):
        print("Sim publishers have been initialized")

def initThrusters(output_type = "real", debug = False):
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
    if (output_type == "simulation"):
        for i in range(NUM_THRUSTERS):
            if (i < 4):
                thrusters[i] = Thruster(i, i, 0, "simulation")
            else:
                thrusters[i] = Thruster(i, i-4, 1,"simulation")
        for i in range(NUM_THRUSTERS):
            rospy.Subscriber("/mantaray/thrusters/"+ str(i) + "/input", Float64, thrusters[i].thrusterCallback)
    elif (output_type == "real"):
        thrusters[0] = Thruster(0, 2, 0, "real", limitAccel=False)
        thrusters[1] = Thruster(1, 1, 0, "real", limitAccel=False)
        thrusters[2] = Thruster(2, 4, 0, "real", limitAccel=False)
        thrusters[3] = Thruster(3, 3, 0, "real", limitAccel=False)
        thrusters[4] = Thruster(4, 3, 1, "real", limitAccel=True)
        thrusters[5] = Thruster(5, 1, 1, "real", limitAccel=True)
        thrusters[6] = Thruster(6, 2, 1, "real", limitAccel=True)
        thrusters[7] = Thruster(7, 4, 1, "real", limitAccel=True)
        init_thrusts = [k for k in range(-15, 15, 1)]
        stopping_thrusts = [k for k in range(15, 0, -1)]
        for j in init_thrusts:
            for i in range(NUM_THRUSTERS):
                thrusters[i].setTargetThrust(j)
                thrusters[i].update()
                time.sleep(0.02)
        # time.sleep(0.5)
        for j in stopping_thrusts:
            for i in range(NUM_THRUSTERS):
                thrusters[i].setTargetThrust(j)
                thrusters[i].update()
                time.sleep(0.02)
        for i in range(NUM_THRUSTERS):
            rospy.Subscriber("/mantaray/thruster/"+ str(i) + "/input", Float64, thrusters[i].thrusterCallback)
            if debug:
                print("Thruster " + str(i) + " has been initialized")
    if debug:
        print("Thrusters have been initialized")

# def simple_thrusters_test():
    # initPcas(addresses=[0x40, 0x41],debug = True)
    # # print()
    # initThrusters(debug = True)

if __name__ == "__main__":
    rospy.init_node("thruster_controller", anonymous=False, log_level=rospy.DEBUG)
    output_type = rospy.get_param("/output_type", default="real")
    if (output_type == "real"):
        import busio
        import board
        from adafruit_pca9685 import PCA9685
        import pandas as pd
        from mantaray_rpi.msg import FloatStamped
        initPcas(addresses=[0x40, 0x41], debug = True)
    elif (output_type == "simulation"):
        from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
        from std_msgs.msg import Float64
        initPubs(debug=True)
        
    initThrusters(output_type, debug = True)
    while not rospy.is_shutdown():
        if(updatingThrusters):
            for i in range(NUM_THRUSTERS):
                thrusters[i].update()
                # print(thrusters[i].currentThrust)