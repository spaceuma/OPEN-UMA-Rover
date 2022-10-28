#Python API for ROS2
import rclpy 
from rclpy.node import Node

#Thread Library for rclpy spin
import threading

# Type of messages published on different topics
#from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import serial #To use install: pip install pyserial
import time
import numpy as np
import math

# Initialization of the array where the speeds received from the topics will be stored
my_data = [0, 0]
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
        #Wheels distance
        #self.declare_parameter('L',0.258)
        #Create subscription
        self.speed = self.create_subscription(Twist,'cmd_vel',self.Callback,1)

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
        print("[Velocity command received]" + "\n")
        print("Linear velocity = " + str(self.v) + " m/s" + "\n")
        print("Angular velocity = " + str(self.w) + " rad/s"+ "\n")
        print("Wheel Velocity Left: " + str(my_data[0]) + " cm/s" + "\n")
        print("Wheel Velocity Right: " + str(my_data[1]) + " cm/s" + "\n")



def main(args=None):
    rclpy.init(args=args)
    rov_node = open_uma_rov_node()
    #Create a thread to constantly spin the node while constantly executing the while loop at the same time
    thread = threading.Thread(target=rclpy.spin, args=(rov_node, ), daemon=True)
    thread.start()
    rate = rov_node.create_rate(20)

    # Beginning of ROS main loop
    while rclpy.ok():

        print("---ROS2 UMA ROV---")
        
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

        # 0.1 second pause
        # time.sleep(0.1)

        # Sending the second speed
        arduino.write(byte_array_2)

        # 0.2 seconds pause
        # time.sleep(0.2)

        # Buffer cleaning
        arduino.flush()

        rate.sleep()

        #Clears de Buffer. Due to changes in Arduino 2.0, .flush() no longer does this but rather waits for the write to be done
        # reset_input_buffer allows us to do this.
        arduino.reset_input_buffer()

    rov_node.destroy_node()
    rclpy.shutdown()
    #Closes Thread activity
    thread.join()

# Catching possible interruptions which can be thrown by rospy.sleep() when Ctrl-C is pressed or
# the Node is otherwise shutdown
if __name__ == '__main__':
    main()
