#!/usr/bin/python
"""!
Main GUI for Arm lab
"""
import sys, logging
import numpy as np
from functools import partial
from PyQt5.QtCore import QThread, Qt, pyqtSlot
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow
from resource.ui import Ui_MainWindow
from piper_arm import PiperArm, PiperArmThread
from camera import Camera, VideoThread
from state_machine import StateMachine, StateMachineThread
import rclpy


""" Radians to/from  Degrees conversions """
D2R = np.pi / 180.0
R2D = 180.0 / np.pi

logging.basicConfig(
    level=logging.INFO,  # Set the log level
    format="%(asctime)s - %(levelname)s - %(message)s",  # Log format
    handlers=[
        logging.StreamHandler(),  # Log to terminal
        logging.FileHandler("control_station.log"),  # Log to a file
    ]
)

class Gui(QMainWindow):
    """!
    Main GUI Class

    Contains the main function and interfaces between the GUI and functions.
    """
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        """ Groups of ui commonents """
        self.joint_readouts = [
            self.ui.rdoutJ1coords,
            self.ui.rdoutJ2coords,
            self.ui.rdoutJ3coords,
            self.ui.rdoutJ4coords,
            self.ui.rdoutJ5coords,
            self.ui.rdoutJ6coords
        ]
        self.joint_slider_rdouts = [
            self.ui.rdoutJ1,
            self.ui.rdoutJ2,
            self.ui.rdoutJ3,
            self.ui.rdoutJ4,
            self.ui.rdoutJ5,
            self.ui.rdoutJ6
        ]
        self.joint_sliders = [
            self.ui.sldrJ1,
            self.ui.sldrJ2,
            self.ui.sldrJ3,
            self.ui.sldrJ4,
            self.ui.sldrJ5,
            self.ui.sldrJ6
        ]
        """Objects Using Other Classes"""
        self.camera = Camera()
        print("Creating piper arm...")
        self.arm = PiperArm()     # This is the entry point where arm instance is created  
        print("Done creating piper arm instance.")
        self.sm = StateMachine(self.arm, self.camera)
        """
        Attach Functions to Buttons & Sliders
        """
        # Video
        self.ui.videoDisplay.setMouseTracking(True)
        self.ui.videoDisplay.mouseMoveEvent = self.trackMouse
        self.ui.videoDisplay.mousePressEvent = self.calibrateMousePress

        # Buttons
        nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state)

        self.ui.btn_estop.clicked.connect(partial(nxt_if_arm_init, 'estop'))
        self.ui.btn_init_arm.clicked.connect(self.initArm)
        # (Dev) TODO: Replace torq_off with something safer
        # self.ui.btn_torq_off.clicked.connect(
        #     lambda: self.rxarm.disable_torque())
        self.ui.btn_torq_on.clicked.connect(lambda: self.arm.enable_fun(True))
        self.ui.btn_sleep_arm.clicked.connect(lambda: self.arm.sleep())
        self.ui.btn_calibrate.clicked.connect(partial(nxt_if_arm_init, 'calibrate'))

        # User Buttons
        # (Student) TODO: Add more lines here to add more buttons
        # To make a button activate a state, copy the lines for btnUser3 but change 'execute' to whichever state you want
        self.ui.btnUser1.setText('Open Gripper')
        self.ui.btnUser1.clicked.connect(lambda: self.arm.gripper_open())
        self.ui.btnUser2.setText('Close Gripper')
        self.ui.btnUser2.clicked.connect(lambda: self.arm.gripper_close())
        self.ui.btnUser3.setText('Execute')
        self.ui.btnUser3.clicked.connect(partial(nxt_if_arm_init, 'execute'))

        # Sliders
        for sldr in self.joint_sliders:
            sldr.valueChanged.connect(self.sliderChange)
        self.ui.sldrSpeed.valueChanged.connect(self.sliderChange)

        # Direct Control
        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)

        # Status
        self.ui.rdoutStatus.setText("Waiting for input")

        """initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)

        """Setup Threads"""
        # State machine
        self.StateMachineThread = StateMachineThread(self.sm)
        self.StateMachineThread.updateStatusMessage.connect(
            self.updateStatusMessage)
        self.StateMachineThread.start()
        self.VideoThread = VideoThread(self.camera)
        self.VideoThread.updateFrame.connect(self.setImage)
        self.VideoThread.start()
        self.ArmThread = PiperArmThread(self.arm)
        self.ArmThread.updateJointReadout.connect(self.updateJointReadout)
        self.ArmThread.updateEndEffectorReadout.connect(
            self.updateEndEffectorReadout)
        self.ArmThread.start()

    """ Slots attach callback functions to signals emitted from threads"""

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)

    @pyqtSlot(list)
    def updateJointReadout(self, joints):
        for rdout, joint in zip(self.joint_readouts, joints):
            rdout.setText(str('%+.2f' % (joint)))

    # Distances should be in mm
    @pyqtSlot(list)
    def updateEndEffectorReadout(self, pos):
        self.ui.rdoutX.setText(str("%+.2f mm" % (pos[0])))
        self.ui.rdoutY.setText(str("%+.2f mm" % (pos[1])))
        self.ui.rdoutZ.setText(str("%+.2f mm" % (pos[2])))
        self.ui.rdoutPhi.setText(str("%+.2f deg" % (pos[3])))
        self.ui.rdoutTheta.setText(str("%+.2f deg" % (pos[4])))
        self.ui.rdoutPsi.setText(str("%+.2f deg" % (pos[5])))

    @pyqtSlot(QImage, QImage, QImage, QImage)
    def setImage(self, rgb_image, depth_image, tag_image, grid_image):
        """!
        @brief      Display the images from the camera.

        @param      rgb_image    The rgb image
        @param      depth_image  The depth image
        """
        if (self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(rgb_image))
        if (self.ui.radioDepth.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_image))
        if (self.ui.radioApriltag.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(tag_image))
        if (self.ui.radioUsr2.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(grid_image))

    """ Other callback functions attached to GUI elements"""
    def sliderChange(self):
        """!
        @brief Slider changed

        Function to change the slider labels when sliders are moved and to command the arm to the given position
        """
        for rdout, sldr in zip(self.joint_slider_rdouts, self.joint_sliders):
            rdout.setText(str(sldr.value()))

        self.ui.rdoutSpeed.setText(
            str(self.ui.sldrSpeed.value()))
        self.arm.set_speed_rate(self.ui.sldrSpeed.value())
    
        # Do nothing if the arm is not initialized
        if self.arm.initialized:
            joint_positions = np.array(
                [sldr.value() for sldr in self.joint_sliders])
            self.arm.set_positions(joint_positions[0:self.arm.num_joints])

    def directControlChk(self, state):
        """!
        @brief      Changes to direct control mode

                    Will only work if the arm is initialized.

        @param      state  State of the checkbox
        """
        if state == Qt.Checked and self.arm.initialized:
            # Go to manual and enable sliders
            self.sm.set_next_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        else:
            # Lock sliders and go to idle
            self.sm.set_next_state("idle")
            self.ui.SliderFrame.setEnabled(False)
            self.ui.chk_directcontrol.setChecked(False)

    def trackMouse(self, mouse_event):
        """!
        @brief      Show the mouse position in GUI

                    TODO: after implementing workspace calibration display the world coordinates the mouse points to in the RGB
                    video image.

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """

        # TODO: Modify this function to change the mouseover text.
        # You should make the mouseover text display the (x, y, z) coordinates of the pixel being hovered over

        pt = mouse_event.pos()
        if self.camera.DepthFrameRaw.any() != 0:
            z = self.camera.DepthFrameRaw[pt.y()][pt.x()]
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" %
                                             (pt.x(), pt.y(), z))
            self.ui.rdoutMouseWorld.setText("(-,-,-)")

    def calibrateMousePress(self, mouse_event):
        """!
        @brief Record mouse click positions for calibration

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """
        """ Get mouse posiiton """
        pt = mouse_event.pos()
        self.camera.last_click[0] = pt.x()
        self.camera.last_click[1] = pt.y()
        self.camera.new_click = True

    def initArm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.ui.SliderFrame.setEnabled(False)
        self.ui.chk_directcontrol.setChecked(False)
        self.sm.set_next_state('initialize_arm')

def main():
    """!
    @brief      Starts the GUI
    """
    try:
        rclpy.init()
        app = QApplication(sys.argv)
        app_window = Gui()
        app_window.show()

        # Set thread priorities
        app_window.VideoThread.setPriority(QThread.HighPriority)
        app_window.ArmThread.setPriority(QThread.NormalPriority)
        app_window.StateMachineThread.setPriority(QThread.LowPriority)

        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("Ctrl+C detected. Shutting down...")
    finally:
        # (DEV) TODO: prevent ctrl+c accident, add sleep() here?
        app_window.close()  # Trigger closeEvent
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
