import numpy as np
import socket
import time
from pyniryo import *

"""
This class object can be used to create socket objects to listen to Sensor Messages 
from mobile devices using websockets (at present uses UDP protocol). The class can 
create robot controller for Niryo robot as a nested object and has algorithms to 
calculate orientation from raw sensor values 
"""

class NiryoMobileController:
    def __init__(self):
        # IMU Data Structutre
        """
        Sensor data structure (Expand here in future if more types of sensors are needed)
        """
        self.imu_data = {
            'readSuccess' : False,
            'timestamp': 0.0,
            'sensorCount': 0,
            'deltaT' : 0.0,
            'setGyroAngles': False,
            'acc_data': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0
            },
            'gyro_data': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0
            },
            'orientation_gyro' : {
                'roll': 0.0,
                'pitch': 0.0,
                'yaw': 0.0
            },
            'orientation_acc' : {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
            },
            'orientation_fused' : {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
            }
        }

        self.robot_ip_address = "10.10.10.10"
        self.localIP     = "10.10.10.145"
        self.localPort   = 5555 
        self.bufferSize  = 1024
        self.niryoAlive = False
        self.deviceCalibrated = False

    """
    Initilialises the Niryo robot and mobile device socket connection
    """
    def initialize(self):
        try:
            # Create a datagram socket
            self.UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

            # Bind to address and ip
            self.UDPServerSocket.bind((self.localIP, self.localPort))
            print("UDP Socket Successfully Created")
        except Exception as error:
            print(error)

        try:
            # Connect to robot & calibrate
            self.robot = NiryoRobot(self.robot_ip_address)
            self.niryoAlive = True
        except Exception as error:
            print(error)
        
        if(self.niryoAlive):
            self.robot.calibrate_auto()
            # Move joints to home
            self.robot.move_to_home_pose()

            # Move to start position
            # self.robot.move_joints(0.01,-0.41,-1.15,0.03,0.03,-0.01)


    """
    Reads the sensor data from the device (add any required protocols here in future (e.g., MQTT, TCP, UCP, WIRED))
    """
    def readSensorData(self):
        bytesAddressPair = self.UDPServerSocket.recvfrom(self.bufferSize)
        message = bytesAddressPair[0]
        address = bytesAddressPair[1]

        # clientMsg = "Message from Client:{}".format(message)
        # clientIP  = "Client IP Address:{}".format(address)
        
        decoded_msg = message.decode('utf-8')
        decoded_msg = decoded_msg.split(',')

        for index, data in enumerate(decoded_msg):
            decoded_msg[index] = float(data)

        if(len(decoded_msg) == 9):
            self.imu_data['deltaT'] = decoded_msg[0] - self.imu_data['timestamp']
            self.imu_data['timestamp'] = decoded_msg[0]

            self.imu_data['acc_data']['x'] = decoded_msg[2]
            self.imu_data['acc_data']['y'] = decoded_msg[3]
            self.imu_data['acc_data']['z'] = decoded_msg[4]

            self.imu_data['gyro_data']['x'] = decoded_msg[6]
            self.imu_data['gyro_data']['y'] = decoded_msg[7]
            self.imu_data['gyro_data']['z'] = decoded_msg[8]

            self.imu_data['readSuccess'] = True
        else:
            self.imu_data['readSuccess'] = False


    """
    Calibrates the sensor data from the mobile device
    """
    def calibrateSensors(self):
        print("Starting sensor device calibration in 5 seconds, please do not move the device")
        for sec in range(0, 5):
            print(5 - sec)
            time.sleep(sec)
        
        self.gyro_sum_x = 0.0
        self.gyro_sum_y = 0.0
        self.gyro_sum_z = 0.0

        print("Calibration Begins")
        for count in range(0, 2000):
            self.readSensorData()
            self.gyro_sum_x += self.imu_data['gyro_data']['x']
            self.gyro_sum_y += self.imu_data['gyro_data']['y']
            self.gyro_sum_z += self.imu_data['gyro_data']['z']
            if(count % 100 == 0):
                print("Completed: {}".format(count/2000))
        
        self.imu_data['gyro_offsets']['x'] = self.gyro_sum_x / 2000.0
        self.imu_data['gyro_offsets']['y'] = self.gyro_sum_y / 2000.0
        self.imu_data['gyro_offsets']['z'] = self.gyro_sum_z / 2000.0

        print("Calibration successful, offsets are: ")
        print("X offset: {}".format(self.imu_data['gyro_offsets']['x']))
        print("Y offset: {}".format(self.imu_data['gyro_offsets']['y']))
        print("Z offset: {}".format(self.imu_data['gyro_offsets']['z']))

    """
    Calculates the orientation of the sensor device using the sensor fusion of Gyro and Accel data 
    """
    def calculateOrientation(self):
        if(self.imu_data['readSuccess']):
            # In mobile, roll is right left tilt
            # GYROSCOPE SENSOR ANGLE CALCULATION
            if(0.0 < self.imu_data['deltaT'] < 0.5):
                self.imu_data['orientation_gyro']['roll'] += self.imu_data['gyro_data']['y'] * self.imu_data['deltaT']
                self.imu_data['orientation_gyro']['pitch'] += self.imu_data['gyro_data']['x'] * self.imu_data['deltaT']

            # ACCELEROMETER SENSOR ANGLE CALCULATION
            acc_total_vector = np.sqrt(np.power(self.imu_data['acc_data']['x'], 2) + np.power(self.imu_data['acc_data']['y'], 2) + np.power(self.imu_data['acc_data']['z'], 2))
            self.imu_data['orientation_acc']['roll'] = -np.arcsin(self.imu_data['acc_data']['x'] / acc_total_vector)
            self.imu_data['orientation_acc']['pitch'] = -np.arcsin(self.imu_data['acc_data']['y'] / acc_total_vector)

            # Initial angle setting
            roll_angle = 0.0
            pitch_angle = 0.0

            if(self.imu_data['setGyroAngles']):
                roll_angle = self.imu_data['orientation_gyro']['roll'] * 0.996 + self.imu_data['orientation_acc']['roll'] * 0.004
                pitch_angle = self.imu_data['orientation_gyro']['pitch'] * 0.996 + self.imu_data['orientation_acc']['pitch'] * 0.004
            else:
                roll_angle = self.imu_data['orientation_acc']['roll']
                pitch_angle = self.imu_data['orientation_acc']['pitch']
                self.imu_data['setGyroAngles'] = True
            
            # Dampening the angles using a complimentary filter
            self.imu_data['orientation_fused']['roll'] = self.imu_data['orientation_fused']['roll'] * 0.9 + roll_angle * 0.1
            self.imu_data['orientation_fused']['pitch'] = self.imu_data['orientation_fused']['pitch'] * 0.9 + pitch_angle * 0.1
        else:
            print("Read Failed during Orientation Calculation")

    def controlNiryo(self):
        if(self.niryoAlive):
            x_move_dist = 0 # Distance in mm here
            y_move_dist = 0
            if(self.imu_data['orientation_fused']['roll']/np.pi * 180.0 > 10.0):
                x_move_dist = 5/1000
            elif(self.imu_data['orientation_fused']['roll']/np.pi * 180.0 < -10.0):
                x_move_dist = -5/1000
            else:
                x_move_dist = 0

            if(self.imu_data['orientation_fused']['pitch']/np.pi * 180.0 > 10.0):
                y_move_dist = 5/1000
            elif(self.imu_data['orientation_fused']['pitch']/np.pi * 180.0 < -10.0):
                y_move_dist = -5/1000
            else:
                y_move_dist = 0

            # print(self.imu_data['orientation_fused']['roll']/np.pi * 180.0, x_move_dist, self.imu_data['orientation_fused']['pitch']/np.pi * 180.0, y_move_dist)

            # if(x_move_dist >= 0.001 or x_move_dist <= -0.001):
            #     try:
            #         # self.robot.jog_pose(x_move_dist, y_move_dist,0,0,0,0)
            #     except Exception as error:
            #         print(error)

    def controlNiryoJoints(self):
        if(self.niryoAlive):
            base_angle = 0.0
            if(self.imu_data['orientation_fused']['roll']/np.pi * 180.0 > 10.0):
                base_angle = 0.01
            elif(self.imu_data['orientation_fused']['roll']/np.pi * 180.0 < -10.0):
                base_angle = -0.01
            else:
                base_angle = 0.0
            joints_angles = self.robot.get_joints()
            base_joint_angle = joints_angles[0] + base_angle

            print(base_joint_angle)
            self.robot.move_joints(base_joint_angle, 0.0, 0.0, 0.0, 0.0, 0.0)
