import rospy
import sys
from std_msgs.msg import Bool
from crazy_msgs.msg import status

class arbiter_node:
    def __init__(self):
        #Init Subscribers
        rospy.loginfo("Creating Subscriber")
        self.launch_sub = rospy.Subscriber("/launch_ready", Bool, self.launch_callback)
        self.task_sub = rospy.Subscriber("/task_ready", Bool, self.task_callback)
        self.land_sub = rospy.Subscriber("/land_ready", Bool, self.land_callback)
        self.ranger_sub = rospy.Subscriber("/ranger_ready", Bool, self.ranger_callback)
        self.status_sub = rospy.Subscriber("/system_status", status, self.status_callback)
        #Init Publishers
        rospy.loginfo("Creating Publishers")
        self.statusPub = rospy.Publisher("/system_status", status, queue_size=10)

        #Init Variable Status
        self.launch_status = True
        self.task_status = True
        self.land_status = True
        self.ranger_status = False
        self.system_status = 'INITIALIZING'

    def launch_callback(self,msg):
        rospy.loginfo("Received Launch Status")
        self.launch_status = msg.data
        rospy.loginfo("Launch Callback Done")
    def task_callback(self,msg):
        rospy.loginfo("Received Task Status")
        self.task_status = msg.data
        rospy.loginfo("Task Callback Done")
    def land_callback(self,msg):
        rospy.loginfo("Received Land Status")
        self.land_status = msg.data
        rospy.loginfo("Land Callback Done")
    def ranger_callback(self,msg):
        rospy.loginfo("Received Ranger Status")
        self.ranger_status = msg.data
        rospy.loginfo("Ranger Callback Done")
    def status_callback(self,msg):
        rospy.loginfo("Received System Status")
        self.system_status = msg.status
        rospy.loginfo("System Status Callback Done")
    

def main(args):
    rospy.loginfo("Creating Arbiter Node")
    node = arbiter_node()
    rospy.init_node('arbiter_node', anonymous=False)
    rate = rospy.Rate(10)
    # try:
    #     rospy.loginfo("Starting to Spin")
    #     rospy.spin()
    #     rospy.loginfo("Spinning")
    # except KeyboardInterrupt:
    #     print("Shutting Down")
    rospy.loginfo("Publishing System Stating") #INITIALIZING, READY, LAUNCHING, IN_AIR, ON_TASK, TASK_COMPLETE, LANDING, DONE
    while not rospy.is_shutdown():
        rospy.loginfo('Publishing System Status')
        try:
            if node.system_status in ['INITIALIZING', 'READY']:
                if node.launch_status and node.task_status and node.land_status and node.ranger_status:
                    rospy.loginfo('Publishing READY Status')
                    statusMsg = status()
                    statusMsg.status = 'READY'
                    node.statusPub.publish(statusMsg)
                else:
                    rospy.loginfo('Publishing INITIALIZING Status')
                    statusMsg = status()
                    statusMsg.status = 'INITIALIZING'
                    node.statusPub.publish(statusMsg)
            rate.sleep()
        except KeyboardInterrupt:
            print("Shutting Down")

if __name__ == '__main__':
    main(sys.argv)
