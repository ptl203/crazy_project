#!/usr/bin/env python
import rospy
from crazyswarm.msg import GenericLogData

def mock_sensor():
    pub = rospy.Publisher('/cf1/crazylog', GenericLogData, queue_size=10)
    rospy.init_node('mock_sensor', anonymous=False)
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        msg = GenericLogData()
        msg.values = [0.0, 0.0, 0.0, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1]
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        mock_sensor()
    except rospy.ROSInterruptException:
        pass