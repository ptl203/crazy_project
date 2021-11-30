#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Bool
from crazy_msgs.msg import ranger, stateEstimate
from crazyswarm.msg import GenericLogData

class sensor_interface:
    def __init__(self):

        #Init Subscribers
        rospy.loginfo("Creating Subscriber")
        self.sub = rospy.Subscriber("/cf1/crazylog", GenericLogData, self.callback)
        #Init Publishers
        rospy.loginfo("Creating Publishers")
        self.statePub = rospy.Publisher("/stateEstimate_data", stateEstimate, queue_size=10)
        self.rangerPub = rospy.Publisher("/ranger_data", ranger, queue_size=10)
        self.statusPub = rospy.Publisher("/ranger_ready", Bool, queue_size=10)

    def callback(self,data):
        rospy.loginfo("Publishing State Data")
        stateMsg = stateEstimate()
        stateMsg.x = data.values[0]
        stateMsg.y = data.values[1]
        stateMsg.z = data.values[2]
        self.statePub.publish(stateMsg)

        rospy.loginfo("Publishing Ranger Data")
        rangerMsg = ranger()
        rangerMsg.front = data.values[3]
        rangerMsg.back = data.values[4]
        rangerMsg.up = data.values[5]
        rangerMsg.down = data.values[6]
        rangerMsg.left = data.values[7]
        rangerMsg.right = data.values[8]
        self.rangerPub.publish(rangerMsg)

        rospy.loginfo("Publish that we are still getting Data")
        statusMessage = Bool()
        statusMessage.data = True
        self.statusPub.publish(statusMessage)

        rospy.loginfo("Callback Done")

def main(args):
    rospy.loginfo("Creating Node")
    node = sensor_interface()
    rospy.init_node('sensor_interface', anonymous=False)
    try:
        rospy.loginfo("Spinning")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == '__main__':
    main(sys.argv)

