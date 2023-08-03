#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float64MultiArray
#from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from mantaray_rpi.msg import FloatStamped
NUM_THRUSTERS = 8
enable_joys = False # Enables the Joy Sticks
saturation = 60
pubs = [None] * 8

def remote_callback(data): # Implement soft kill switch
    global pubs
    rospy.logdebug("manual_callback")
    joy_data = data.data
    # if (joy_data[16]):
    #     soft_kill_switch.publish(Bool(True))
    # elif(joy_data[17]):
    #     soft_kill_switch.publish(Bool(False))
    left_mult = joy_data[3]
    right_mult = joy_data[7]
    if joy_data[18]:
        left_mult *= -1
    if joy_data[19]:
        right_mult *= -1
    thruster_tots = [0] * NUM_THRUSTERS

    thruster_tots[0] += joy_data[8] * right_mult * saturation
    thruster_tots[1] += joy_data[9] * right_mult * saturation
    thruster_tots[2] += joy_data[10] * right_mult * saturation
    thruster_tots[3] += joy_data[11] * right_mult * saturation
    if (joy_data[14] > 0):
        thruster_tots[4] += joy_data[14] * left_mult * saturation
    else:
        thruster_tots[6] -= joy_data[14] * left_mult * saturation
    if (joy_data[13] < 0):
        thruster_tots[5] -= joy_data[13] * left_mult * saturation
    else:
        thruster_tots[7] += joy_data[13] * left_mult * saturation

    # Game commands
    if (enable_joys): # 0.866 is sqrt(3)/2 since the thrusters are rotated 30deg
        rospy.loginfo("Inside mode == game")
        if (joy_data[2]):
            #Left and right
            thruster_tots[0]+=(-joy_data[0] * left_mult * saturation)
            thruster_tots[1]+=(joy_data[0] * left_mult * saturation)
            thruster_tots[2]+=(joy_data[0] * left_mult * saturation)
            thruster_tots[3]+=(-joy_data[0] * left_mult * saturation)
            #Forward and backward
            thruster_tots[0]+=(-joy_data[1] * left_mult * saturation)
            thruster_tots[1]+=(-joy_data[1] * left_mult * saturation)
            thruster_tots[2]+=(joy_data[1] * left_mult * saturation)
            thruster_tots[3]+=(joy_data[1] * left_mult * saturation)
        else:
            # Yaw rotation
            thruster_tots[0]+=((joy_data[1] * 0.866 - joy_data[0] * 0.5) * left_mult * saturation)
            thruster_tots[1]+=((joy_data[1] * 0.866 + joy_data[0] * 0.5) * left_mult * saturation)
            thruster_tots[2]+=((joy_data[1] * 0.866 + joy_data[0] * 0.5) * left_mult * saturation)
            thruster_tots[3]+=((joy_data[1] * 0.866 - joy_data[0] * 0.5) * left_mult * saturation)
            # thruster_inputs[0].set_thrust((joy_data[1] * 0.866 - joy_data[0] * 0.5)/1.366 * left_mult * saturation)
            # thruster_inputs[1].set_thrust((joy_data[1] * 0.866 + joy_data[0] * 0.5)/1.366 * left_mult * saturation)
            # thruster_inputs[2].set_thrust((joy_data[1] * 0.866 + joy_data[0] * 0.5)/1.366 * left_mult * saturation)
            # thruster_inputs[3].set_thrust((joy_data[1] * 0.866 - joy_data[0] * 0.5)/1.366 * left_mult * saturation)
        if (joy_data[6]):
            # Need to enable the PID and change the rosparam to fit the right_mult
            pass
        else:
            thruster_tots[4]+=((-joy_data[4] - joy_data[5]) * right_mult * saturation)
            thruster_tots[5]+=((joy_data[4] - joy_data[5]) * right_mult * saturation)
            thruster_tots[6]+=((-joy_data[4] + joy_data[5]) * right_mult * saturation)
            thruster_tots[7]+=((joy_data[4] + joy_data[5]) * right_mult * saturation)
            # thruster_inputs[4].set_thrust((-joy_data[4] - joy_data[5])/2 * right_mult * thrusters[4].saturation)
            # thruster_inputs[5].set_thrust((joy_data[4] - joy_data[5])/2 * right_mult * thrusters[5].saturation)
            # thruster_inputs[6].set_thrust((-joy_data[4] + joy_data[5])/2 * right_mult * thrusters[6].saturation)
            # thruster_inputs[7].set_thrust((joy_data[4] + joy_data[5])/2 * right_mult * thrusters[7].saturation)
    for i in range(NUM_THRUSTERS):
        num = Float64(max(min(saturation, thruster_tots[i]), -saturation))
        rospy.logdebug("Publishing to thruster_inputs[%i]: %s", i, thruster_tots[i])
        pubs[i].publish(num)

if __name__ == "__main__":
    rospy.init_node("joy2thrust", anonymous=False)
    for i in range(NUM_THRUSTERS):
        pubs[i] = rospy.Publisher("/mantaray/thruster/"+str(i)+"/input", Float64, queue_size=2)
    rospy.Subscriber("/mantaray/remote_joy", Float64MultiArray, queue_size=1, callback=remote_callback)
    rospy.spin()
