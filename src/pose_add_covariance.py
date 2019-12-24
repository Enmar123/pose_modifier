#!/usr/bin/python


import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class RosNode:
    def __init__(self):
        print("starting node: pose_add_covariance")
        rospy.init_node('pose_add_covariance')
        
        pose_topic_in  = rospy.get_param('~pose_topic_in')
        pose_topic_out = rospy.get_param('~pose_topic_out')
        
        self.define_msg()
        
        ps_sub  = rospy.Subscriber(pose_topic_in, PoseStamped, self.callback)
        self.pwcs_pub = rospy.Publisher(pose_topic_out, PoseWithCovarianceStamped , queue_size=10)
            
        rospy.spin()
    
    def define_msg(self):
        self.pwcs_msg = PoseWithCovarianceStamped()
        self.pwcs_msg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]
        
    def callback(self, ps_msg):
        self.pwcs_msg.header = ps_msg.header
        self.pwcs_msg.pose.pose = ps_msg.pose
        
        self.pwcs_pub.publish(self.pwcs_msg)
        

if __name__ == '__main__':
    try:
        node = RosNode()
    except rospy.ROSInterruptException:
        pass 