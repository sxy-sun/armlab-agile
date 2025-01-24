import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QThread, pyqtSignal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from piper_sdk import *
import numpy as np
from scipy.spatial.transform import Rotation as R
import math, time

class PiperArm(Node):
    def __init__(self):
        """!
        Initializes the PiperArm class using Piper SDK.
        """
        super().__init__('piper_arm_node')
        self.num_joints = 6         # gripper not included
        self.piper = C_PiperInterface(can_name='can0')
        self.piper.ConnectPort()

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.ee_pose_pub = self.create_publisher(Pose, 'ee_pose', 10)

        # Joint state information
        self.joint_state = JointState()
        self.joint_state.name = [f'joint{i}' for i in range(self.num_joints)]
        self.joint_state.position = [0.0] * self.num_joints

        self.initialized = False

    def initialize(self):
        """ Initializes the Piper arm. """
        self.piper.EnableArm(7)
        time.sleep(0.1)
        if not self.enable_arm():
            raise RuntimeError("Failed to enable arm.")
        self.gripper_open()  # Release gripper
        # self.set_positions([0.0] * self.num_joints)  # Reset all joints to initial position
        self.initialized = True
        self.get_logger().info("Piper Arm initialized and reset to initial position.")

    def enable_arm(self):
        enable_flag = False
        timeout = 5
        start_time = time.time()
        while not enable_flag:
            elapsed_time = time.time() - start_time
            enable_flag = all([
                self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status,
            ])
            if enable_flag:
                self.get_logger().info("All motors enabled.")
            elif elapsed_time > timeout:
                self.get_logger().error("Motor enable timeout.")
                return False
            time.sleep(1)
        return True

    def disable(self):
        """ Disables the Piper arm. """
        self.piper.DisableArm(7)
        self.initialized = False
        self.get_logger().info("Piper Arm disabled.")

    def gripper_open(self):
        """ Opens the gripper to its maximum extent. """
        gripper_angle = 80 # degree
        self.piper.GripperCtrl(gripper_angle*1000, gripper_effort=800, gripper_code=0x01, set_zero=0)
        self.wait_for_motion_complete()
        self.get_logger().info("Gripper opened.")

    def gripper_close(self):
        """ Closes the gripper with specified effort. """
        gripper_angle = 10
        self.piper.GripperCtrl(gripper_angle*1000, gripper_effort=800, gripper_code=0x01, set_zero=0)
        self.wait_for_motion_complete()
        self.get_logger().info(f"Gripper closed.")

    def set_positions(self, joint_positions):
        """ Sets joint positions. 
            The arm expects angle in unit 0.001 degrees.
            multiply 180/pi is from radians to degrees, 1000 is scaling factor.
            so when we set position, we set angle in radians
        """
        joint_positions = [int(pos * 180 / math.pi * 1000) for pos in joint_positions]
        self.piper.JointCtrl(*joint_positions)
        self.get_logger().info(f"Set positions: {joint_positions}")
        self.wait_for_motion_complete()

    def publish_joint_states(self):
        """ Publishes joint states. """
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = [
            self.piper.GetArmJointMsgs().joint_state.joint_1 / 1000.0,
            self.piper.GetArmJointMsgs().joint_state.joint_2 / 1000.0,
            self.piper.GetArmJointMsgs().joint_state.joint_3 / 1000.0,
            self.piper.GetArmJointMsgs().joint_state.joint_4 / 1000.0,
            self.piper.GetArmJointMsgs().joint_state.joint_5 / 1000.0,
            self.piper.GetArmJointMsgs().joint_state.joint_6 / 1000.0,
        ]
        self.joint_state_pub.publish(self.joint_state)

    def publish_ee_pose(self):
        """ Publishes end-effector pose. """
        pose = Pose()
        end_pose = self.piper.GetArmEndPoseMsgs().end_pose
        pose.position.x = end_pose.X_axis / 1000000
        pose.position.y = end_pose.Y_axis / 1000000
        pose.position.z = end_pose.Z_axis / 1000000

        roll = math.radians(end_pose.RX_axis / 1000)
        pitch = math.radians(end_pose.RY_axis / 1000)
        yaw = math.radians(end_pose.RZ_axis / 1000)
        quaternion = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion

        self.ee_pose_pub.publish(pose)

    def wait_for_motion_complete(self, timeout=10):
        """ Waits until the arm completes its current motion or the timeout is reached. """
        time.sleep(0.1) # give motion status some time to update
        start_time = time.time()
        while True:
            elapsed_time = time.time() - start_time
            motion_status = self.piper.GetArmStatus().arm_status.motion_status
            if motion_status == 0:  # Assuming 0 means motion complete
                # self.get_logger().info("Motion completed.")
                return True
            if elapsed_time > timeout:
                self.get_logger().error("Motion timeout.")
                return False
            time.sleep(0.1)  # Avoid busy-waiting

class PiperArmThread(QThread):
    """ Thread for running Piper Arm operations. """
    updateJointReadout = pyqtSignal(list)
    updateEndEffectorReadout = pyqtSignal(list)

    def __init__(self, piper_arm, parent=None):
        super().__init__(parent=parent)
        self.piper_arm = piper_arm
        self.node = rclpy.create_node('piper_arm_thread')
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def run(self):
        """ Spins the executor for handling callbacks. """
        try:
            while rclpy.ok():
                self.piper_arm.publish_joint_states()
                self.piper_arm.publish_ee_pose()
                self.executor.spin_once(timeout_sec=0.02)
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

    def callback(self, data):
        positions = [p / 1000.0 for p in self.piper_arm.joint_state.position]
        self.updateJointReadout.emit(positions)
        ee_pose = self.piper_arm.publish_ee_pose()
        self.updateEndEffectorReadout.emit([ee_pose.position.x, ee_pose.position.y, ee_pose.position.z])

if __name__ == '__main__':
    rclpy.init()
    arm = PiperArm()
    arm_thread = PiperArmThread(arm)
    arm_thread.start()
    try:
        arm.piper.MotionCtrl_2(0x01, 0x01, 20, 0x00)
        arm.initialize()

        arm.piper.MotionCtrl_2(0x01, 0x01, 20, 0x00)
        arm.set_positions([0, 1, -1.57, 0, 0, 1])
        arm.gripper_close()

        arm.piper.MotionCtrl_2(0x01, 0x01, 20, 0x00)
        arm.set_positions([0, 0, 0, 0, 0.39, 0])
        arm.gripper_open()

        rclpy.spin(arm)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Exiting...")
    finally:
        print("Shutting down...")
        arm.disable()  # Disable the arm
        arm_thread.quit()  # Signal the thread to stop
        arm_thread.wait()  # Wait for the thread to terminate
        if rclpy.ok():  # Check if the context is still valid
            rclpy.shutdown()  # Shutdown ROS2
        print("Clean exit.")
