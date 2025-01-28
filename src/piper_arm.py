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
        is_enabled = self.enable_fun(True)  # Enable all motors
        self.gripper_open()  
        if is_enabled:
            self.initialized = True
            logger.info("The Piper arm is enabled!")
        else:
            logger.error("The Piper arm failed to be enabled!")

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
        self.GripperCtrl(gripper_angle*1000, gripper_effort=800, gripper_code=0x01, set_zero=0)
        self.wait_for_motion_complete()
        logger.info("Gripper opened.")

    def gripper_close(self):
        """ Closes the gripper with specified effort. """
        gripper_angle = 10
        self.GripperCtrl(gripper_angle*1000, gripper_effort=800, gripper_code=0x01, set_zero=0)
        self.wait_for_motion_complete()
        logger.info("Gripper closed.")

    def set_positions(self, joint_positions):
        """ Sets joint positions. 
            The arm expects angle in unit 0.001 degrees.
            multiply 180/pi is from radians to degrees, 1000 is scaling factor.
            so when we set position, we set angle in radians
        """
        joint_positions = [int(pos * 180 / math.pi * 1000) for pos in joint_positions]
        self.JointCtrl(*joint_positions)
        logger.info(f"Set positions: {joint_positions}")
        self.wait_for_motion_complete()

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

class PiperArmThread(QThread):
    """ Thread for running Piper Arm operations. """
    updateJointReadout = pyqtSignal(list)
    updateEndEffectorReadout = pyqtSignal(list)

    def __init__(self, piper_arm, parent=None):
        super().__init__(parent=parent)
        self.arm = piper_arm

    def run(self):
        #TODO
        while True:
            joint_poses = self.arm.GetArmJointMsgs()
            ee_pose = self.arm.GetArmEndPoseMsgs()
            self.updateJointReadout.emit()
            self.updateEndEffectorReadout.emit([ee_pose.position.x, ee_pose.position.y, ee_pose.position.z])

if __name__ == '__main__':
    # Configure logging only if run standalone
    if not logging.getLogger().hasHandlers():
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - %(message)s",
        )
    arm = PiperArm()
    # arm_thread = PiperArmThread(arm)
    # arm_thread.start()
    try:
        arm.MotionCtrl_2(0x01, 0x01, 20, 0x00)
        arm.initialize()

        arm.MotionCtrl_2(0x01, 0x01, 20, 0x00)
        arm.set_positions([0, 1, -1.57, 0, 0, 1])
        arm.gripper_close()

        arm.MotionCtrl_2(0x01, 0x01, 20, 0x00)
        arm.set_positions([0, 0, 0, 0, 0.39, 0])
        arm.gripper_open()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Exiting...")
    finally:
        print("Shutting down...")
        arm.enable_fun(False)  # Disable the arm
        # arm_thread.quit()  # Signal the thread to stop
        # arm_thread.wait()  # Wait for the thread to terminate
        print("Clean exit.")
