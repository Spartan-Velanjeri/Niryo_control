import socket
import time
import numpy as np

localIP     = "10.10.10.145"
localPort   = 5555
bufferSize  = 1024
calibrated = False
time_counting = False

# IMU Data Structutre
imu_data = {
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

# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))

print("UDP server up and listening")

# Listen for incoming datagrams
while(True):
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    address = bytesAddressPair[1]

    clientMsg = "Message from Client:{}".format(message)
    clientIP  = "Client IP Address:{}".format(address)
    
    decoded_msg = message.decode('utf-8')
    decoded_msg = decoded_msg.split(',')

    for index, data in enumerate(decoded_msg):
        decoded_msg[index] = float(data)

    if(len(decoded_msg) == 9):
        imu_data['deltaT'] = decoded_msg[0] - imu_data['timestamp']
        imu_data['timestamp'] = decoded_msg[0]

        imu_data['acc_data']['x'] = decoded_msg[2]
        imu_data['acc_data']['y'] = decoded_msg[3]
        imu_data['acc_data']['z'] = decoded_msg[4]

        imu_data['gyro_data']['x'] = decoded_msg[6]
        imu_data['gyro_data']['y'] = decoded_msg[7]
        imu_data['gyro_data']['z'] = decoded_msg[8]

        # In mobile, roll is right left tilt
        # GYROSCOPE SENSOR ANGLE CALCULATION
        if(0.0 < imu_data['deltaT'] < 0.5):
            imu_data['orientation_gyro']['roll'] += imu_data['gyro_data']['y'] * imu_data['deltaT'] 

        # ACCELEROMETER SENSOR ANGLE CALCULATION
        acc_total_vector = np.sqrt(np.power(imu_data['acc_data']['x'], 2) + np.power(imu_data['acc_data']['y'], 2) + np.power(imu_data['acc_data']['z'], 2))
        imu_data['orientation_acc']['roll'] = -np.arcsin(imu_data['acc_data']['x'] / acc_total_vector)

        # Initial angle setting
        roll_angle = 0.0
        pitch_angle = 0.0

        if(imu_data['setGyroAngles']):
            roll_angle = imu_data['orientation_gyro']['roll'] * 0.996 + imu_data['orientation_acc']['roll'] * 0.004
        else:
            roll_angle = imu_data['orientation_acc']['roll']
            imu_data['setGyroAngles'] = True
        
        # Dampening the angles using a complimentary filter
        imu_data['orientation_fused']['roll'] = imu_data['orientation_fused']['roll'] * 0.9 + roll_angle * 0.1

    # Calibrating gyroscope 
    # if(~calibrated):
    #     print("Starting gyroscope calibration in 5 seconds, please do not move the device")
    #     for time_count in range(0, 5):
    #         print(time_count)
    #         time.sleep(time_count)

    print(imu_data['orientation_fused']['roll']/np.pi * 180.0)
