#! /usr/bin/env python
import rospy
from kortex_driver.srv import PlayJointTrajectory, PlayJointTrajectoryRequest
from kortex_driver.msg import JointAngle
from trajectory_msgs.msg import JointTrajectory
import copy


def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)

    client = rospy.ServiceProxy("/my_gen3/base/play_joint_trajectory", PlayJointTrajectory)

    request = PlayJointTrajectoryRequest()

    joint_position = [JointAngle() for i in range(7)]
    # joint_position = JointAngles[]
    for i, j in enumerate(joint_position):
        j.joint_identifier = i + 1

    # msg = rospy.wait_for_message("/planned_trajectory", JointTrajectory)

    for i in range(7): 
        if msg.points[0].positions[i]<0 :
            joint_position[i].value = msg.points[0].positions[i]*180.0/3.141592653 + 360
        
        else :
            joint_position[i].value = msg.points[0].positions[i]*180.0/3.141592653
        

        print(joint_position[i])

    request.input.joint_angles.joint_angles = joint_position

    result = client(request)
    print(result)

    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/planned_trajectory", JointTrajectory, callback)

    rospy.spin()

if __name__ == '__main__':

    listener()
