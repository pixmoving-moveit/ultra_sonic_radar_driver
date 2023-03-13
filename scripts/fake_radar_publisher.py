#! /usr/bin/python

import rospy
from can_msgs.msg import Frame

def fakeCanMessagePublish():
    pub = rospy.Publisher('/received_messages', Frame, queue_size=5)
    rospy.init_node('fake_radar_publisher_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        fake_can_frame_0 = Frame()
        fake_can_frame_0.header.stamp = rospy.Time.now()
        fake_can_frame_0.dlc = 8
        fake_can_frame_0.id = 0x611
        fake_can_frame_0.is_extended = False
        fake_can_frame_0.data = [0x23, 0x32, 0x40, 0x41, 0x24, 0x33, 0x11, 0x02]
        pub.publish(fake_can_frame_0)
        
        fake_can_frame_1 = Frame()
        fake_can_frame_1.header.stamp = rospy.Time.now()
        fake_can_frame_1.dlc = 8
        fake_can_frame_1.id = 0x612
        fake_can_frame_1.is_extended = False
        fake_can_frame_1.data = [0x23, 0x32, 0x40, 0x41, 0x24, 0x33, 0x11, 0x02]
        pub.publish(fake_can_frame_1)
        rate.sleep()
        
if __name__ == "__main__":
    try:
        fakeCanMessagePublish()
    except rospy.ROSInterruptException:
        pass