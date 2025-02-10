# Test piper arm

![](https://robosavvy.s3.eu-west-1.amazonaws.com/products/Agilex/Piper/02.webp)

## Get Ready

1. virtual env: `python3 -m venv piper_env`
2. activate the env `source piper_env/bin/activate`
3. install everything:

```bash

pip3 install python-can
pip3 install scipy
pip3 install piper_sdk
```
```bash
pip install 'numpy<2'
pip3 install PyQt5
pip3 install opencv-python
pip3 install future modern_robotics
pip uninstall opencv-python
pip install opencv-python-headless
```

## Test
1. Activate CAN `./can_activate.sh`
2. Run piper_arm.py as a standalone python program: `python3 piper_arm.py`. Without modify the code, the arm will perform IK and goes to a position.
3. Run the control station: `./launch_armlab.sh` and then `./launch_control_Station.sh`
