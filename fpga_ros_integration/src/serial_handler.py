#!/usr/bin/env python3
import rospy
import rospkg
import serial
#from geometry_msgs.msg import Point
from fpga_ros_integration.msg import fpga_data
from std_msgs.msg import Int16
from std_msgs.msg import String

ser = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

def parse_value(bit):
    if bit == 1:
        return bit
    elif bit == 3:
        return -1
    else:
        return 0

current_score = 0
def get_score(score):
    global current_score
    current_score = score.data
    rospy.loginfo(score)

def ser_pub():
    rospy.init_node("serial_node", anonymous = False)
    serial_pub = rospy.Publisher("fpga/serial_data", fpga_data, queue_size=100)
    serial_pub_bin = rospy.Publisher("fpga/serial_data_bin", String, queue_size=100)
    score_sub = rospy.Subscriber("fpga/score/bin", String, get_score)
    rate = rospy.Rate(100)

    rospy.loginfo("Serial node online...")
    while not rospy.is_shutdown():

        raw_byte = bin(int.from_bytes(ser.read(), byteorder='big'))[2:].zfill(8)
        byte = int(raw_byte, 2)
        byte_str = str(raw_byte)
        #byte = int(b'00000000', 2)

        serial_data = fpga_data()
        
        #if byte != b'00000000':
        serial_data.differential.z = byte & 1
        serial_data.differential.y = parse_value((byte & 6) >> 1)
        serial_data.differential.x = parse_value((byte & 24) >> 3)
        serial_data.action_mux = (byte & 96) >> 5
        serial_data.action_trigger = (byte & 128) >> 7

        serial_pub.publish(serial_data)

        serial_bin_data = String()
        serial_bin_data.data = str(byte_str)
        serial_pub_bin.publish(serial_bin_data)

        rate.sleep()

if __name__ == '__main__':

    #rospy.loginfo("Serial connection open") 
    #print("Pene")

    #while True:
        #print("Pene2")
    try: 
            #ser = "Hola"
            #ser.open()
            #rospy.loginfo("Serial connection open")
        ser_pub()
    except rospy.ROSInterruptException:
        print("Pene3")
        ser.close()
        rospy.loginfo("Serial connection closed")
    except serial.SerialException:
        #print("Pene4")
        rospy.loginfo("Port not available")
            #break
