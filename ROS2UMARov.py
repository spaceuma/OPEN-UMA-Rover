#Python API for ROS2
import rclpy 
from rclpy.node import Node

#Thread Library for rclpy spin
import threading

# Type of messages published on different topics
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import serial #To use install: pip install pyserial
import time
import numpy as np
import math

# Print control variables #
#Set to 1 to visualise the info. related to the velocity sent from this node
joystick_vel = 0
#Set to 1 to visualise the info. related to the encoders sent from the Rover
encoder_info = 1

# Initialization of the array where the speeds received from the topics will be stored
my_data = [0, 0]

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

# Principal node
class open_uma_rov_node(Node):

    def __init__(self):
        super().__init__('rov_node')
        #Create subscription and publisher
        self.speed = self.create_subscription(Twist,'cmd_vel',self.Callback,10)
        #Publishers for wheels data
        self.publisher_wheel1 = self.create_publisher(Float32, 'Pulses_Wheel1', 10)
        self.publisher_wheel2 = self.create_publisher(Float32, 'Pulses_Wheel2', 10)
        self.publisher_velwheel1 = self.create_publisher(Float32, 'Vel_pulses1', 10) 
        self.publisher_velwheel2 = self.create_publisher(Float32, 'Vel_pulses2', 10)
        # self.current = self.create_publisher(Float32, 'current', 10)

    #Recibe la informacion de velocidad    
    def Callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        # Calculate velocities for each wheel
        L = 0.258 #Wheels distance
        my_data[0] = ((1.0 * self.v - self.w * L / 2)*100) #Left
        my_data[1] = ((1.0 * self.v + self.w * L / 2)*100) #Right
        #La informacion que se debe recibir de Nav2 es en m/s pero el archivo OpenUMARov.ino trabaja en cm/s.
        #Debido a ello se recibe una consigna en m/s, que al ser operada es multiplicada por 100 para que la v de cada rueda sea cm/s.
        #La información de Nav2 se recibe en formato de velocidad {x,y,theta}, donde se limita y=0 por ser vehiculo no omnidireccional.
        if joystick_vel == 1:
            print("[Velocity command received]" + "\n")
            print("Linear velocity = " + str(self.v) + " m/s" + "\n")
            print("Angular velocity = " + str(self.w) + " rad/s"+ "\n")
            print("Wheel Velocity Left: " + str(my_data[0]) + " cm/s" + "\n")
            print("Wheel Velocity Right: " + str(my_data[1]) + " cm/s" + "\n")



def main(args=None):
    rclpy.init(args=args)
    rov_node = open_uma_rov_node()
    #Create a thread to constantly spin the node while constantly executing the loop at the same time
    thread = threading.Thread(target=rclpy.spin, args=(rov_node, ), daemon=True)
    thread.start()
    rate = rov_node.create_rate(10)
    #Publisher data
    msg_wheels = Float32()

    # Beginning of ROS main loop
    while rclpy.ok():

        print("--ROS2UMAROV--")

        # Storing in two different variables data from the topic
        speed_1 = my_data[0]
        speed_2 = my_data[1]

        # print("First speed (Left) is: " + str(speed_1) + " cm/s" + "\n")
        # print("Seconds speed (Right) is: " + str(speed_2) + " cm/s" + "\n")

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

        # Sending the first speed
        arduino.write(byte_array_1)

        # Cleaning the buffer before sending data
        arduino.flush()
        
        # Sending the second speed
        arduino.write(byte_array_2)

        # Waiting data to be sent
        arduino.flush()
        
        # if sentence that verifies Arduino is sending something through serial port
        if arduino.inWaiting():
            # Loop to get all data from the serial port in the correct order
            for i in range(inputs_number):

                # Reading from the serial port
                arduino_value = arduino.readline().rstrip()
                print(arduino_value)
                # If byte does not hold info. it will set the byte to zero.
                if len(arduino_value) == 0 or arduino_value == "b'-'":
                    # We will use the value from the last iteration if the variable has no data
                    pass
                else:
                    # Storing data from the serial port
                    vector_received_values[i] = arduino_value

            # Number of pulses of the first wheel
            pulses_1 = vector_received_values[0]
            msg_wheels.data = pulses_1
            rov_node.publisher_wheel1.publish(msg_wheels)

            # Calculation of the distance of the first wheel
            distance_1 = (float(pulses_1) * 0.03) * (37.7 / 360)

            # Number of pulses of the second wheel
            pulses_2 = vector_received_values[1]
            msg_wheels.data = pulses_2
            rov_node.publisher_wheel2.publish(msg_wheels)

            # Calculation of the distance of the second wheel
            distance_2 = (float(pulses_2) * 0.03) * (37.7 / 360)

            # Difference of pulses of the first wheel in 10 ms
            speed_pulses_1 = vector_received_values[2]
            msg_wheels.data = speed_pulses_1
            rov_node.publisher_velwheel1.publish(msg_wheels)

            # Calculation of the first speed
            speed_wheel_1 = (((float(speed_pulses_1) * 0.03) * (37.7 / 360)) / 10) * 1000

            # Difference of pulses of the second wheel in 10 ms
            speed_pulses_2 = vector_received_values[3]
            msg_wheels.data = speed_pulses_2
            rov_node.publisher_velwheel2.publish(msg_wheels)

            # Calculation of the first speed
            speed_wheel_2 = (((float(speed_pulses_2) * 0.03) * (37.7 / 360)) / 10) * 1000

            # Number of pulses to keep a constant speed in wheel 1
            ref_1 = vector_received_values[4]

            # Calculation of the first motor current
            current_1 = ((float(ref_1) * (5 / 1024)) - 2.516) / (-0.22)

            # Number of pulses to keep a constant speed in wheel 2
            ref_2 = vector_received_values[5]

            # Calculation of the second motor current
            current_2 = ((float(ref_2) * (5 / 1024)) - 2.513) / (-0.17)

            # Calculation of the total current
            total_current = current_1 + current_2
            # msg_wheels.data = total_current
            # rov_node.current.publish(msg_wheels)

            if encoder_info == 1:
                print("First wheel speed: " + str(speed_wheel_1) + " cm/s")
                print("Second wheel speed: " + str(speed_wheel_2) + " cm/s")

        # Buffer cleaning
        arduino.reset_input_buffer()

        rate.sleep()

    rov_node.destroy_node()
    rclpy.shutdown()
    #Closes Thread activity
    thread.join()

# Catching possible interruptions which can be thrown by rospy.sleep() when Ctrl-C is pressed or
# the Node is otherwise shutdown
if __name__ == '__main__':
    main()
