#!/usr/bin/env python
import rospy
import sys
import time
from std_msgs.msg import Bool
from crazy_msgs.msg import ranger, status

class launch_node:
    def __init__(self):

        #Set Variables
        self.system_trigger = False
        self.ranger_trigger = False
        self.ranger_count = 0

        #Init Subscribers
        rospy.loginfo("Creating Subscribers")
        self.status_sub = rospy.Subscriber("/system_status", status, self.status_callback)
        self.ranger_sub = rospy.Subscriber("/ranger_data", ranger, self.ranger_callback)
        
        #Init Publishers
        rospy.loginfo("Creating Publishers")
        self.system_pub = rospy.Publisher("/system_status", status, queue_size=10)
        self.land_status_pub = rospy.Publisher("/launch_ready", Bool, queue_size=10)

    def status_callback(self,data):
        #rospy.loginfo("status_callback called")
        if data.status == "READY":
            #rospy.loginfo("System is Ready, turning system_trigger True")
            self.system_trigger = True
        else:
            pass
            #rospy.loginfo("System either initializing or mission has started")

    def ranger_callback(self,data):
        rospy.loginfo("ranger_callback called")
        if data.up < 508.0:
            rospy.loginfo("incrementing ranger_count")
            self.ranger_count += 1
        if self.ranger_count > 10:
            rospy.loginfo("setting ranger_trigger true")
            self.ranger_trigger = True


def main(args):
    rospy.loginfo("Creating Launch Node")
    node = launch_node()
    rospy.init_node('launch_node', anonymous=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            #rospy.loginfo("Checking System and Ranger Triggers") 
            #rospy.loginfo("Publish that we are alive")
            ready_msg = Bool()
            ready_msg.data = True
            node.land_status_pub.publish(ready_msg)
            rospy.loginfo("Ranger Trigger is %s and System Trigger is %s", node.ranger_trigger, node.system_trigger)
            if node.system_trigger and node.ranger_trigger:
                rospy.loginfo("system and ranger trigger true! Taking Off!")
                rospy.loginfo("Setting System Status to LAUNCHING")
                status_msg = status()
                status_msg.status = "LAUNCHING"
                node.system_pub.publish(status_msg)
                rospy.loginfo("Status Updated")
                rospy.loginfo("For Now Sleep Ten")
                time.sleep(10)
                rospy.loginfo("Set Status to IN_AIR")
                status_msg = status()
                status_msg.status = "IN_AIR"
                node.system_pub.publish(status_msg)
                rospy.loginfo("set system_trigger to False")
                node.system_trigger = False
            else:
                rospy.loginfo("Triggers are False")
            
            rate.sleep()
        except KeyboardInterrupt:
            print("Shutting Down")

if __name__ == '__main__':
    main(sys.argv)