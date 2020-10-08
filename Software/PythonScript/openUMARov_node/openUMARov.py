#!/usr/bin/env python

# Library that allows writing the code with Python
import rospy

# Type of messages published on different topics
from std_msgs.msg import Float32

import serial

import time

import numpy as np

import math

# Initialization of the array where the speeds received from the topics will be stored
my_data = [0, 0]

# Initialization of all the variables that will be received from the GPS
latitude = [0]

longitude = [0]

height = [0]

position_covariance_0 = [0]

position_covariance_4 = [0]

position_covariance_8 = [0]

position_covariance_type = [0]

status = [0]

status_service = [0]

# Number of data that will be received from the serial port
inputs_number = 6

# Creation of a zeros vector to store the data from the serial port
vector_received_values = np.zeros(inputs_number)

# Creation of the serial object
port = '/dev/ttyACM0'
baudrate = 115200
bytesize = serial.EIGHTBITS
parity = serial.PARITY_NONE
stopbits = serial.STOPBITS_ONE
timeout = 0
xonxoff = False
rtscts = False
write_timeout = 0

# Starting serial communication
arduino = serial.Serial(port, baudrate, bytesize, parity, stopbits, timeout, xonxoff, rtscts, write_timeout)

# 1 second break waiting for Arduino and Raspberry to connect
time.sleep(1)


# Function to receive the first speed from the topic openUMARov_speed_1
def topic_speed_1(data):
    # Data from the topic is stored in the first position of my_data
    my_data[0] = data.data

    return my_data


# Function to receive the second speed from the topic openUMARov_speed_2
def topic_speed_2(data):
    # Data from the topic is stored in the second position of my_data
    my_data[1] = data.data

    return my_data


# Function to receive the latitude from de topic nmeaGPS_latitude
def topic_latitude(data):
    # Data from the topic is stored in the variable latitude
    latitude[0] = data.data

    return latitude


# Function to receive the longitud from de topic nmeaGPS_longitude
def topic_longitude(data):
    # Data from the topic is stored in the variable longitude
    longitude[0] = data.data

    return longitude


# Function to receive the latitude from de topic nmeaGPS_height
def topic_height(data):
    # Data from the topic is stored in the variable height
    height[0] = data.data

    return height


# Function to receive the latitude from de topic nmeaGPS_position_covariance_0
def topic_position_covariance_0(data):
    # Data from the topic is stored in the variable position_covariance_0
    position_covariance_0[0] = data.data

    return position_covariance_0


# Function to receive the latitude from de topic nmeaGPS_position_covariance_4
def topic_position_covariance_4(data):
    # Data from the topic is stored in the variable position_covariance_4
    position_covariance_4[0] = data.data

    return position_covariance_4


# Function to receive the latitude from de topic nmeaGPS_position_covariance_8
def topic_position_covariance_8(data):
    # Data from the topic is stored in the variable position_covariance_8
    position_covariance_8[0] = data.data

    return position_covariance_8


# Function to receive the latitude from de topic nmeaGPS_position_covariance_type
def topic_position_covariance_type(data):
    # Data from the topic is stored in the variable position_covariance_type
    position_covariance_type[0] = data.data

    return position_covariance_type


# Function to receive the latitude from de topic nmeaGPS_status
def topic_status(data):
    # Data from the topic is stored in the variable status
    status[0] = data.data

    return status


# Function to receive the latitude from de topic nmeaGPS_status_service
def topic_status_service(data):
    # Data from the topic is stored in the variable status_service
    status_service[0] = data.data

    return status_service


# Principal node
def open_uma_rov_node():
    # Telling rospy the node name
    rospy.init_node('open_uma_rov_node', anonymous=True)

    # ROS loop speed: 10 hz (10 times per second)
    rospy.Rate(10)

    # Subscription to the topic where the user will publish the first speed
    rospy.Subscriber('openUMARov_speed_1', Float32, topic_speed_1)

    # Subscription to the topic where the user will publish the second speed
    rospy.Subscriber('openUMARov_speed_2', Float32, topic_speed_2)

    # Subscription to the topic where the GPS will publish the latitude
    rospy.Subscriber('nmeaGPS_latitude', Float32, topic_latitude)

    # Subscription to the topic where the GPS will publish the longitude
    rospy.Subscriber('nmeaGPS_longitude', Float32, topic_longitude)

    # Subscription to the topic where the GPS will publish the height
    rospy.Subscriber('nmeaGPS_height', Float32, topic_height)

    # Subscription to the topic where the GPS will publish the position_covariance_0
    rospy.Subscriber('nmeaGPS_position_covariance_0', Float32, topic_position_covariance_0)

    # Subscription to the topic where the GPS will publish the position_covariance_4
    rospy.Subscriber('nmeaGPS_position_covariance_4', Float32, topic_position_covariance_4)

    # Subscription to the topic where the GPS will publish the position_covariance_8
    rospy.Subscriber('nmeaGPS_position_covariance_8', Float32, topic_position_covariance_8)

    # Subscription to the topic where the GPS will publish the position_covariance_type
    rospy.Subscriber('nmeaGPS_position_covariance_type', Float32, topic_position_covariance_type)

    # Subscription to the topic where the GPS will publish the status
    rospy.Subscriber('nmeaGPS_status', Float32, topic_status)

    # Subscription to the topic where the GPS will publish the status service
    rospy.Subscriber('nmeaGPS_status_service', Float32, topic_status_service)

    # Publishing to different topics:

    # Declaration that the node is publishing to the topic openUMARov_distance_1 using the message type Float32
    publish_distance_1 = rospy.Publisher('openUMARov_distance_1', Float32, queue_size=10)

    # Declaration that the node is publishing to the topic openUMARov_distance_2 using the message type Float32
    publish_distance_2 = rospy.Publisher('openUMARov_distance_2', Float32, queue_size=10)

    # Declaration that the node is publishing to the topic openUMARov_speed_wheel_1 using the message type Float32
    publish_speed_wheel_1 = rospy.Publisher('openUMARov_speed_wheel_1', Float32, queue_size=10)

    # Declaration that the node is publishing to the topic openUMARov_speed_wheel_2 using the message type Float32
    publish_speed_wheel_2 = rospy.Publisher('openUMARov_speed_wheel_2', Float32, queue_size=10)

    # Declaration that the node is publishing to the topic openUMARov_current_1 using the message type Float32
    publish_current_1 = rospy.Publisher('openUMARov_current_1', Float32, queue_size=10)

    # Declaration that the node is publishing to the topic openUMARov_current_2 using the message type Float32
    publish_current_2 = rospy.Publisher('openUMARov_current_2', Float32, queue_size=10)

    # Declaration that the node is publishing to the topic openUMARov_total_current using the message type Float32
    publish_total_current = rospy.Publisher('openUMARov_total_current', Float32, queue_size=10)

    # Beginning of ROS main loop
    while not rospy.is_shutdown():

        print("OPEN UMA ROV ")

        # Storing in two different variables data from the topic
        speed_1 = my_data[0]
        speed_2 = my_data[1]

        print("First speed is: " + str(speed_1) + "\n")

        print("Seconds speed is: " + str(speed_2) + "\n")

        # Calculation of the pulses
        speed_1_pulses = speed_1 * (0.01 / (0.03 * (37.7 / 360)))

        speed_2_pulses = speed_2 * (0.01 / (0.03 * (37.7 / 360)))

        # Rounding pulses values
        speed_1_pulses_rounded = int(math.ceil(speed_1_pulses))

        speed_2_pulses_rounded = int(math.ceil(speed_2_pulses))

        # Creating two byte arrays, one for each data that will be sent through serial port, where each speed
        # will be converted to an integer of 16 bits: this means that each speed will occupy 2 positions in the array,
        # sending through serial port a total of 4 bytes
        byte_array_1 = np.array([speed_1_pulses_rounded], dtype=np.int16)

        byte_array_2 = np.array([speed_2_pulses_rounded], dtype=np.int16)

        # Cleaning the buffer before sending data
        arduino.flush()

        # Sending the first speed
        arduino.write(byte_array_1)

        # 0.1 second pause
        time.sleep(0.1)

        # Sending the second speed
        arduino.write(byte_array_2)

        # 0.2 seconds pause
        time.sleep(0.2)

        # Buffer cleaning
        arduino.flush()

        # if sentence that verifies Arduino is sending something through serial port
        if arduino.inWaiting():

            # Loop to get all data from the serial port in the correct order
            for i in range(inputs_number):

                # Reading from the serial port
                arduino_value = arduino.readline().rstrip()

                # Storing data from the serial port
                vector_received_values[i] = arduino_value

            print("All data have been received.")

            # Getting the values received from serial port printed to screen:

            # Number of pulses of the first wheel
            pulses_1 = vector_received_values[0]

            print("The number of the first wheel pulses is: ")
            print(pulses_1)

            # Calculation of the distance of the first wheel
            distance_1 = (float(pulses_1) * 0.03) * (37.7 / 360)

            print("First wheel distance (cm) is: ")

            # Getting the message printed to screen, written to the Node's log file, and written to rosout
            rospy.loginfo(float(distance_1))

            # Publication of the message to the topic
            publish_distance_1.publish(float(distance_1))

            # Number of pulses of the second wheel
            pulses_2 = vector_received_values[1]

            print("The number of the second wheel pulses is: ")

            print(pulses_2)

            # Calculation of the distance of the second wheel
            distance_2 = (float(pulses_2) * 0.03) * (37.7 / 360)

            print("Second wheel distance (cm) is: ")

            # Getting the message printed to screen, written to the Node's log file, and written to rosout
            rospy.loginfo(float(distance_2))

            # Publication of the message to the topic
            publish_distance_2.publish(float(distance_2))

            # Difference of pulses of the first wheel in 10 ms
            speed_pulses_1 = vector_received_values[2]

            print("The difference of pulses of the first wheel in 10 ms is: ")
            print(speed_pulses_1)

            # Calculation of the first speed
            speed_wheel_1 = (((float(speed_pulses_1) * 0.03) * (37.7 / 360)) / 10) * 1000

            print("First wheel speed (cm / s) is: ")

            # Getting the message printed to screen, written to the Node's log file, and written to rosout
            rospy.loginfo(float(speed_wheel_1))

            # Publication of the message to the topic
            publish_speed_wheel_1.publish(float(speed_wheel_1))

            # Difference of pulses of the second wheel in 10 ms
            speed_pulses_2 = vector_received_values[3]

            print("The difference of the second wheel pulses in 10 ms is: ")
            print(speed_pulses_2)

            # Calculation of the first speed
            speed_wheel_2 = (((float(speed_pulses_2) * 0.03) * (37.7 / 360)) / 10) * 1000

            print("Second wheel speed (cm / s) is: ")

            # Getting the message printed to screen, written to the Node's log file, and written to rosout
            rospy.loginfo(float(speed_wheel_2))

            # Publication of the message to the topic
            publish_speed_wheel_2.publish(float(speed_wheel_2))

            # Number of pulses to keep a constant speed in wheel 1
            ref_1 = vector_received_values[4]

            print("REF1, number of pulses to keep a constant speed in wheel 1, is: ")

            print(ref_1)

            # Calculation of the first motor current
            current_1 = ((float(ref_1) * (5 / 1024)) - 2.516) / (-0.22)

            print("First motor current (A) is: ")

            # Getting the message printed to screen, written to the Node's log file, and written to rosout
            rospy.loginfo(float(current_1))

            # Publication of the message to the topic
            publish_current_1.publish(float(current_1))

            # Number of pulses to keep a constant speed in wheel 2
            ref_2 = vector_received_values[5]

            print("REF2, number of pulses to keep a constant speed in wheel 2, is: ")
            print(ref_2)

            # Calculation of the second motor current
            current_2 = ((float(ref_2) * (5 / 1024)) - 2.513) / (-0.17)

            print("Second motor current (A) is: ")

            # Getting the message printed to screen, written to the Node's log file, and written to rosout
            rospy.loginfo(float(current_2))

            # Publication of the message to the topic
            publish_current_2.publish(float(current_2))

            # Calculation of the total current
            total_current = current_1 + current_2

            print("Total current (A) is: ")

            # Getting the message printed to screen, written to the Node's log file, and written to rosout
            rospy.loginfo(float(total_current))

            # Publication of the message to the topic
            publish_total_current.publish(float(total_current))

            # Publishing values from the GPS
            # Latitude
            print("Rover latitude is: ")

            print(latitude)

            # Longitude
            print("Rover longitude is: ")

            print(longitude)

            # Height
            print("Rover height is: ")

            print(height)

            # Position_covariance_0
            print("Position covariance [0] is: ")

            print(position_covariance_0)

            # Position_covariance_4
            print("Position covariance [4] is: ")

            print(position_covariance_4)

            # Position_covariance_8
            print("Position covariance [8] is: ")

            print(position_covariance_8)

            # Position_covariance_type
            print("Position covariance type is: ")

            print(position_covariance_type)

            # Status
            print("Status is: ")

            print(status)

            # Position_covariance_type
            print("Status service is: ")

            print(status_service)

        print("All values have been published.")


# Catching possible interruptions which can be thrown by rospy.sleep() when Ctrl-C is pressed or
# the Node is otherwise shutdown
if __name__ == '__main__':
    try:
        open_uma_rov_node()
    except rospy.ROSInterruptException:
        pass
