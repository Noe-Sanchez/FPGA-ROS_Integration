#!/usr/bin/env python3
import rospy
import rospkg
import serial
#from geometry_msgs.msg import Point
from fpga_ros_integration.msg import fpga_data
from std_msgs.msg import Int16
from std_msgs.msg import String

current_score = 0
def get_score(score):
    global current_score
    current_score = score.data
    rospy.loginfo(score)

def ser_pub(fpga):
    rospy.init_node("serial_node", anonymous = False)
    serial_pub = rospy.Publisher("fpga/serial_data", fpga_data, queue_size=100)
    score_sub = rospy.Subscriber("fpga/score/bin", String, get_score)
    rate = rospy.Rate(100)

    rospy.loginfo("Serial node online...")
    while not rospy.is_shutdown():

        #byte = bin(int.from_bytes(ser.read(), byteorder='big'))[2:].zfill(8)
        byte = int(b'00000000', 2)

        serial_data = fpga_data()
        
        #if byte != b'00000000':
        serial_data.differential.x = byte & 3
        serial_data.differential.y = (byte & 12) >> 2
        serial_data.differential.z = (byte & 16) >> 4
        serial_data.action_mux = (byte & 96) >> 5
        serial_data.action_trigger = (byte & 128) >> 7

        serial_pub.publish(serial_data)
        rate.sleep()

if __name__ == '__main__':

    """
    ser = serial.Serial(
        port=sys.argv[1],
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    """

    try:
        ser = "Hola"
        #ser.open()
        ser_pub(ser)
    except rospy.ROSInterruptException:
        ser.close()
        rospy.loginfo("Serial connection closed")
        pass
    except serial.SerialException:
        rospy.loginfo("Port not available")
        pass
