#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

def pose_callback(msg, args):
    tf_buffer, pub = args
    try:
        # Look up the transform from the original frame to the torso_lift_link
        trans = tf_buffer.lookup_transform("torso_lift_link", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.1))

        # Apply the transform
        transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, trans)

        # Publish the new pose
        pub.publish(transformed_pose)
    except tf2_ros.TransformException:
        pass

if __name__ == '__main__':
    rospy.init_node('pose_transformer_node')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pub = rospy.Publisher('/object_pose_torso', PoseStamped, queue_size=10)
    rospy.Subscriber('/object_pose', PoseStamped, pose_callback, callback_args=(tf_buffer, pub))

    rospy.spin()