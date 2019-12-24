#!/usr/bin/python


import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class RosNode:
    def __init__(self):
        print("starting node: pose_add_covariance")
        rospy.init_node('pose_add_covariance')
        
        pose_topic_in  = rospy.get_param('~pose_topic_in')
        odom_topic_out = rospy.get_param('~odom_topic_out')
        self.child_frame_id = rospy.get_param('~child_frame_id')
        
        self.define_msg()
        
        self.sub = rospy.Subscriber(pose_topic_in, PoseStamped, self.callback)
        self.pub = rospy.Publisher(odom_topic_out, Odometry, queue_size=10)
            
        rospy.spin()
    
    def define_msg(self):
        self.odom_msg = Odometry()
        
    def callback(self, msg):
        self.odom_msg.header = msg.header
        self.odom_msg.pose.pose = msg.pose
        self.odom_msg.child_frame_id = self.child_frame_id
        
        self.pub.publish(self.odom_msg)
        

if __name__ == '__main__':
    try:
        node = RosNode()
    except rospy.ROSInterruptException:
        pass 