#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class DummyObstacles(Node):
    def __init__(self):
        super().__init__('dummy_obstacle_publisher')
        self.pub = self.create_publisher(MarkerArray, '/opensot/external_collisions', 10)
        self.timer = self.create_timer(1.0, self.publish_obstacles)
        self.get_logger().info("Publishing dummy obstacles to /opensot/external_collisions...")

    def publish_obstacles(self):
        msg = MarkerArray()
        time_now = self.get_clock().now().to_msg()

        # 1. A Box in front of the robot
        box = Marker()
        box.header.frame_id = "opensot/world"
        box.header.stamp = time_now
        box.ns = "test_obstacles"
        box.id = 1
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x = 0.6
        box.pose.position.y = 0.0
        box.pose.position.z = 0.8
        box.pose.orientation.w = 1.0
        box.scale.x = 0.2
        box.scale.y = 0.6
        box.scale.z = 0.2
        box.color.r, box.color.g, box.color.b, box.color.a = 1.0, 0.0, 0.0, 0.8
        msg.markers.append(box)

        # 2. A Cylinder to the left
        cyl = Marker()
        cyl.header.frame_id = "opensot/world"
        cyl.header.stamp = time_now
        cyl.ns = "test_obstacles"
        cyl.id = 2
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD
        cyl.pose.position.x = 0.5
        cyl.pose.position.y = 0.4
        cyl.pose.position.z = 0.5
        cyl.pose.orientation.w = 1.0
        cyl.scale.x = 0.2
        cyl.scale.y = 0.2
        cyl.scale.z = 0.8
        cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = 0.0, 0.0, 1.0, 0.8
        msg.markers.append(cyl)

        # 3. A Tetrahedron (Triangle List) to the right
        tet = Marker()
        tet.header.frame_id = "opensot/world"
        tet.header.stamp = time_now
        tet.ns = "test_obstacles"
        tet.id = 3
        tet.type = Marker.TRIANGLE_LIST
        tet.action = Marker.ADD
        tet.pose.position.x = 0.5
        tet.pose.position.y = -0.4
        tet.pose.position.z = 0.5
        tet.pose.orientation.w = 1.0

        # Scale must be 1.0 for Triangle Lists so points represent exact meters
        tet.scale.x = 1.0
        tet.scale.y = 1.0
        tet.scale.z = 1.0
        tet.color.r, tet.color.g, tet.color.b, tet.color.a = 0.0, 1.0, 0.0, 0.8

        # Define the 4 vertices of a tetrahedron
        p0 = Point(x=0.0, y=0.0, z=0.0)
        p1 = Point(x=0.2, y=0.0, z=0.0)
        p2 = Point(x=0.1, y=0.2, z=0.0)
        p3 = Point(x=0.1, y=0.1, z=0.2)

        # A TRIANGLE_LIST needs 3 points per face. 4 faces = 12 points total.
        tet.points = [
            p0, p2, p1,  # Base
            p0, p1, p3,  # Side 1
            p1, p2, p3,  # Side 2
            p2, p0, p3   # Side 3
        ]
        msg.markers.append(tet)

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = DummyObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()