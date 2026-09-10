#!/usr/bin/env python3

import os
import time

import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


MENAGERIE_ROOT = "/home/forest_ws/external/mujoco_menagerie"
ROBOTS_ROOT = "/home/forest_ws/robots"


MODEL_PATHS = {
    "pro": os.path.join(ROBOTS_ROOT, "pal_tiago_pro", "xmls", "scene_tiago_pro.xml"),
    "dual": os.path.join(MENAGERIE_ROOT, "pal_tiago_dual", "scene_position.xml"),
}


class MujocoSimNode(Node):
    def __init__(self):
        super().__init__("mujoco_sim_node")

        self.declare_parameter("robot_model", "pro")
        self.robot_model = self.get_parameter("robot_model").value

        if self.robot_model not in MODEL_PATHS:
            raise ValueError(
                f"Unsupported robot_model='{self.robot_model}'. Expected 'pro' or 'dual'."
            )

        self.xml_path = MODEL_PATHS[self.robot_model]

        if not os.path.isfile(self.xml_path):
            raise FileNotFoundError(f"MuJoCo XML not found: {self.xml_path}")

        self.get_logger().info(f"Robot model: {self.robot_model}")
        self.get_logger().info(f"Loading MJCF: {self.xml_path}")

        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)


        self.joint_to_qpos = {}
        self.joint_to_qvel = {}
        self.joint_to_actuator = {}

        self._build_model_maps()

        #Subscritions
        self.joint_command_sub = self.create_subscription(
            JointState,
            "/opensot/joint_states",
            self.joint_command_callback,
            10,
        )

        # self.base_command_sub = self.create_subscription(
        #     Twist,
        #     "/opensot/base_velocity_command",
        #     self.base_command_callback,
        #     10,
        # )

        #Publishers 
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )


        mujoco.mj_forward(self.model, self.data)

        self.get_logger().info("MuJoCo model loaded successfully")
        self.get_logger().info(
            f"nq={self.model.nq}, nv={self.model.nv}, nu={self.model.nu}"
        )

    def _build_model_maps(self):
        """Build mappings from joint names to qpos, qvel, and actuator indices."""
        for joint_id in range(self.model.njnt):
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
            )

            if name is None:
                continue

            self.joint_to_qpos[name] = int(self.model.jnt_qposadr[joint_id])
            self.joint_to_qvel[name] = int(self.model.jnt_dofadr[joint_id])

        for actuator_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[actuator_id, 0])

            if joint_id < 0:
                continue

            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
            )

            if name is not None:
                self.joint_to_actuator[name] = actuator_id


    def joint_command_callback(self, msg):
        """Apply incoming joint position commands to the corresponding actuators."""
        for name, target in zip(msg.name, msg.position):
            actuator_id = self.joint_to_actuator.get(name)

            if actuator_id is not None:
                self.data.ctrl[actuator_id] = float(target)

    def publish_joint_states(self):
        """ publish the current mujuco joint positions and velocities as a jointstate message."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for name, qpos_id in self.joint_to_qpos.items():
            qvel_id = self.joint_to_qvel[name]

            msg.name.append(name)
            msg.position.append(float(self.data.qpos[qpos_id]))
            msg.velocity.append(float(self.data.qvel[qvel_id]))

        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()

    try:
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            while rclpy.ok() and viewer.is_running():
                start = time.monotonic()

                rclpy.spin_once(node, timeout_sec=0.0)
                mujoco.mj_step(node.model, node.data)
                node.publish_joint_states()
                viewer.sync()

                elapsed = time.monotonic() - start
                sleep_time = node.model.opt.timestep - elapsed

                if sleep_time > 0.0:
                    time.sleep(sleep_time)

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()