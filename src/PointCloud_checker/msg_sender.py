#!/usr/bin/python
import copy
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from robotic_arm_inspector.msg import planes_msg

last_Kinect_PC2 = PointCloud2()

def callback(data):
    global last_Kinect_PC2
    print('data received')
    last_Kinect_PC2 = copy.deepcopy(data)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():
    pub = rospy.Publisher('planes_check', planes_msg, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber('/camera1/depth/points', PointCloud2, callback)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        msg = planes_msg()
        msg.first_name = 'Gionni'
        msg.last_name = 'Robert'
        msg.pc1 = copy.deepcopy(last_Kinect_PC2)
        print msg.pc1.fields
        hello_str = "hello world " + msg.first_name + " " + msg.last_name
        rospy.loginfo(hello_str)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
