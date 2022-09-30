from pyniryo import *

robot_ip_address = "10.10.10.10"

# Connect to robot & calibrate
robot = NiryoRobot(robot_ip_address)
robot.calibrate_auto()
# Move joints to home
robot.move_to_home_pose()

# Move to start position
robot.move_joints(0.01,-0.41,-1.15,0.03,0.03,-0.01)

# Initiate Orientation values
Roll = 0
Pitch = 0
# Operation begins from here
while True:
    #start_pose = mobile.get_pose()
    Roll = 0
    Pitch = 0
    Roll = int(input('Enter Roll value'))
    while Roll != 0.0 or Pitch != 0.0:
        roll_small = Roll/1000
        print(roll_small)
        robot.jog_pose(roll_small, roll_small,0,0,0,0)

        # After Jog movemnet
        Roll = 0

# Stop TCP connection
robot.close_connection()