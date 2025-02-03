import logging
from PyQt5.QtCore import QThread, pyqtSignal
from piper_sdk import *
import numpy as np
from scipy.spatial.transform import Rotation as R
import math, time

logger = logging.getLogger(__name__)

class PiperArm(C_PiperInterface):
    def __init__(self, can_name='can0'):
        """Initializes the PiperArm class using the Piper SDK."""
        super().__init__(can_name=can_name)  # Initialize the C_PiperInterface part

        self.num_joints = 6  # Number of joints (gripper not included)
        self.ConnectPort()  # Initialize CAN port

        self.speed_rate = 20  # percentage 
        self.initialized = False

    def initialize(self):
        """Initializes the Piper arm."""
        self.MotionCtrl_2(0x01, 0x01, self.speed_rate, 0x00)
        is_enabled = self.enable_fun(True)  # Enable all motors
        if is_enabled:
            self.initialized = True
            logger.info("The Piper arm is enabled!")
        else:
            logger.error("The Piper arm failed to be enabled!")
        self.set_positions([0, 90, -90, 0, 0, 0])
        self.gripper_open()  

    def enable_fun(self, enable:bool):
        enable_flag = False
        loop_flag = False
        timeout = 5
        start_time = time.time()
        while not (loop_flag):
            elapsed_time = time.time() - start_time
            enable_list = []
            enable_list.append(self.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            enable_list.append(self.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
            enable_list.append(self.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
            enable_list.append(self.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
            enable_list.append(self.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
            enable_list.append(self.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)
            if(enable):
                enable_flag = all(enable_list)
                self.EnableArm(7)
                self.GripperCtrl(0,1000,0x01, 0)
            else:
                enable_flag = any(enable_list)
                self.DisableArm(7)
                self.GripperCtrl(0,1000,0x00, 0)
            if(enable_flag == enable):  # Success, break while loop
                loop_flag = True
                enable_flag = True
            else:                       # Fail, re-try
                loop_flag = False       
                enable_flag = False
            if elapsed_time > timeout:
                enable_flag = False
                loop_flag = True
                break
            time.sleep(0.5)
        response = enable_flag
        return response

    def gripper_open(self,):
        """ Opens the gripper to its maximum extent. """
        gripper_angle = 80 # degree
        self.MotionCtrl_2(0x01, 0x01, self.speed_rate, 0x00)
        self.GripperCtrl(gripper_angle*1000, gripper_effort=800, gripper_code=0x01, set_zero=0)
        logger.info("Gripper opened.")

    def gripper_close(self):
        """ Closes the gripper with specified effort. """
        gripper_angle = 10
        self.MotionCtrl_2(0x01, 0x01, self.speed_rate, 0x00)
        self.GripperCtrl(gripper_angle*1000, gripper_effort=800, gripper_code=0x01, set_zero=0)
        logger.info("Gripper closed.")

    def set_positions(self, joint_positions):
        """ Sets joint positions. 
            The arm expects angle in unit 0.001 degrees.
            multiply 1000 is scaling factor.
            When we set position, we set angle in degrees
        """
        self.MotionCtrl_2(0x01, 0x01, self.speed_rate, 0x00)
        logger.info(f"Set positions: {joint_positions}")
        # joint_positions = [int(pos * 180 / math.pi * 1000) for pos in joint_positions]
        joint_positions = [int(pos * 1000) for pos in joint_positions]
        self.JointCtrl(*joint_positions)

    def wait_for_motion_complete(self, timeout=10):
        """ Waits until the arm completes its current motion or the timeout is reached. """
        time.sleep(0.1) # give motion status some time to update
        start_time = time.time()
        while True:
            elapsed_time = time.time() - start_time
            motion_status = self.GetArmStatus().arm_status.motion_status
            if motion_status == 0:  #  Motion complete
                #logger.info("Motion completed.")
                return True
            if elapsed_time > timeout:
                logger.error("Motion timeout.")
                return False
            time.sleep(0.1)  # Avoid busy-waiting
    
    def get_joint_state(self):
        joint_state = self.GetArmJointMsgs().joint_state
        joint_state_lst = [
            joint_state.joint_1,
            joint_state.joint_2,
            joint_state.joint_3,
            joint_state.joint_4,
            joint_state.joint_5,
            joint_state.joint_6,
        ]
        return [round(j * 0.001, 3) for j in joint_state_lst]

    def get_end_effector_pose(self):
        """Forward Kinematics"""
        end_effector_pose = self.GetArmEndPoseMsgs().end_pose
        EE_pose_lst = [
            end_effector_pose.X_axis,
            end_effector_pose.Y_axis,
            end_effector_pose.Z_axis,
            end_effector_pose.RX_axis,
            end_effector_pose.RY_axis,
            end_effector_pose.RZ_axis,
        ]

        # Get Euler angles from the SDK
        EE_pose_lst = [round(pose * 0.001, 3) for pose in EE_pose_lst]
        # rx, ry, rz = EE_pose_lst[3:]

        # Convert to quaternion (assuming ZYX order)
        # quaternion = R.from_euler('zyx', [rz, ry, rx], degrees=True).as_quat()
        # print("Quaternion:", quaternion)
        # Convert back to Euler
        # euler_check = R.from_quat(quaternion).as_euler('zyx', degrees=True)
        # print("Recomputed Euler Angles:", euler_check)
        return EE_pose_lst

    def set_end_effector_pose(self, X, Y, Z, RX, RY, RZ):
        """Inverse Kinematics"""
        self.MotionCtrl_2(0x01, 0x00, self.speed_rate, 0x00)
        self.EndPoseCtrl(X*1000, Y*1000, Z*1000, RX*1000, RY*1000, RZ*1000)
        return 0

    def estop(self):
        self.MotionCtrl_1(0x01,0x03,0) # Emergency Stop
        for i in range(10, 0, -1):
            logger.warning(f"Arm reset in {i} seconds...Hold the arm!!!")
            time.sleep(1)
        self.reset()

    def reset(self):
        time.sleep(0.1)
        self.MotionCtrl_1(0x02,0,0)
        self.MotionCtrl_2(0, 0, 0, 0x00)

    def sleep(self):
        self.MotionCtrl_2(0x01, 0x01, self.speed_rate, 0x00)
        self.set_positions([0, 0, 0, 0, 23, 0])

    def set_speed_rate(self, speed_rate):
        self.speed_rate = speed_rate
        logger.info(f"Set moving speed to {speed_rate}%")

class PiperArmThread(QThread):
    """ Thread for running Piper Arm operations. """
    updateJointReadout = pyqtSignal(list)
    updateEndEffectorReadout = pyqtSignal(list)

    def __init__(self, piper_arm, parent=None):
        super().__init__(parent=parent)
        self.arm = piper_arm
        self.is_running = False

    def run(self):
        self.is_running = True
        while self.is_running:
            try:
                joint_state = self.arm.get_joint_state()
                ee_pose = self.arm.get_end_effector_pose()
                self.updateJointReadout.emit(joint_state)
                self.updateEndEffectorReadout.emit(ee_pose)
                time.sleep(0.01)  # Reduce CPU usage
            except Exception as e:
                logger.error(f"Thread error: {e}")
                break

    def stop(self):
        self.is_running = False  # Signal the loop to exit
        self.wait()  # Wait for thread to finish

if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO,  # Change to DEBUG if needed
        format="%(asctime)s - %(levelname)s - %(message)s",
        handlers=[
            logging.StreamHandler(),  # Ensure logging prints to console
            logging.FileHandler("piper_arm.log", mode='w')  # Optional: Log to file
        ]
    )

    logger.setLevel(logging.DEBUG)  # Set a global logging level

    arm = PiperArm()
    arm_thread = PiperArmThread(arm)
    arm_thread.start()
    try:
        arm.enable_fun(True)
        arm.set_end_effector_pose(170, 10, 350, 73, 27, 72) # IK
        for i in range(5, 0, -1):
            logger.warning(f"sleep for {i} seconds")
            time.sleep(1)
        arm.set_end_effector_pose(170, 10, 300, 73, 27, 72) # IK
        for i in range(5, 0, -1):
            logger.warning(f"sleep for {i} seconds")
            time.sleep(1)

        # # Given quaternion [x, y, z, w]
        # quaternion = [-0.148, -0.769, -0.151, -0.602]

        # # Convert to Euler angles in ZYX order (used by the SDK)
        # euler_angles = R.from_quat(quaternion).as_euler('zyx', degrees=True)
        # # Extract RX, RY, RZ from the output
        # rz, ry, rx =  [int(angle) for angle in euler_angles]

        # print(f"Converted Euler Angles (ZYX): RX={rx:.2f}, RY={ry:.2f}, RZ={rz:.2f}")

        # arm.set_end_effector_pose(52, 0, 166, rx, ry, rz) # IK
        # for i in range(5, 0, -1):
        #     logger.warning(f"sleep for {i} seconds")
        #     time.sleep(1)
        arm.sleep()
        time.sleep(3)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Exiting...")
        arm.estop()
    except Exception as e:
        logger.error(f"Critical error: {e}", exc_info=True)
        arm.estop()
    finally:
        arm.MotionCtrl_1(0, 0, 0x02)  
        print("Shutting down...")
        print("Torque off in 3 seconds...")
        time.sleep(3)
        arm.enable_fun(False)
        arm.reset()
        arm_thread.stop()  # Use custom stop method
        print("Clean exit.")
