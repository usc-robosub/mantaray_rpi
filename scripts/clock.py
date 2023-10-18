#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Clock

def simtime_talker():
    pub1 = rospy.Publisher('clock',Clock, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    sim_speed_multiplier = 10    
    sim_clock = Clock()
    zero_time = rospy.get_time()

    while not rospy.is_shutdown():
       sim_clock.clock = rospy.Time.from_sec(sim_speed_multiplier*(rospy.get_time() - zero_time))
       rospy.loginfo(sim_clock)
       pub1.publish(sim_clock)
       rate.sleep()

if __name__ == '__main__':
    try:
        simtime_talker()
    except rospy.ROSInterruptException:
        pass