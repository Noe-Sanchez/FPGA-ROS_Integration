#!/usr/bin/env python3
import rospy
import rospkg
from math import dist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_msgs.msg import Bool

gates = [[2,0,2],
       [2,2,2],
       [4,2,2],
       [4,4,2],
       [4,4,4],
       [6,4,2],
       [6,6,2],
       [6,6,4],
       [6,6,6],
       [-2,-2,2]]
check = [0,0,0,0,0,0,0,0,0,0]
score = 0

def get_pose(pose):
    global gates
    global check
    global score
    current_pose = [round(pose.pose.position.x, 2), round(pose.pose.position.y, 2), round(pose.pose.position.z, 2)]

    for index, point in enumerate(gates):
        #rospy.loginfo(dist(point, current_pose))
        if check[index] == 0:
            if dist(point, current_pose) < 0.5:
                score = score + (2 ** index)
                check[index] = 1
                rospy.loginfo("Se pasó por el anillo " + str(index) + "! \nPuntuacion actual: " + str(sum(check)))


def get_reset(reset):
    global check
    global score
    if reset.data:
        check = [0,0,0,0,0,0,0,0,0,0]
        score = 0

def reward_pub():
    rospy.init_node("rewarder", anonymous = False)
    score_pub = rospy.Publisher("fpga/score/total", Int16, queue_size=100)
    score_pub_bin = rospy.Publisher("fpga/score/bin", String, queue_size=100)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, get_pose)
    pose_sub = rospy.Subscriber("fpga/reset", Bool, get_reset)
    rate = rospy.Rate(2)

    rospy.loginfo("Rewarder node online...")
    while not rospy.is_shutdown():
        score_pub.publish(score)
        score_pub_bin.publish(format(score, 'b').zfill(8))
        rate.sleep()

if __name__ == '__main__':
    try:
        reward_pub()
    except rospy.ROSInterruptException:
        pass
