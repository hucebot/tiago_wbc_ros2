#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.pub = self.create_publisher(PoseStamped, '/object_pose_torso', 10)
        self.sub = self.create_subscription(PoseStamped, '/object_pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        try:
            # Look up transform
            trans = self.tf_buffer.lookup_transform(
                'torso_lift_link', 
                msg.header.frame_id, 
                rclpy.time.Time()
            )
            
            # Apply transform to the pose itself
            transformed_pose = tf2_geometry_msgs.do_transform_pose(msg.pose, trans)
            
            # Construct and publish the new PoseStamped
            new_msg = PoseStamped()
            new_msg.header.stamp = self.get_clock().now().to_msg()
            new_msg.header.frame_id = 'torso_lift_link'
            new_msg.pose = transformed_pose
            
            self.pub.publish(new_msg)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()