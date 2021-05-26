#! /usr/bin/env python
import rospy
from kortex_driver.srv import PlayJointTrajectory, PlayJointTrajectoryRequest
from kortex_driver.msg import JointAngle
from trajectory_msgs.msg import JointTrajectory
import copy

rospy.init_node('haha', anonymous=True)
client = rospy.ServiceProxy("/my_gen3/base/play_joint_trajectory", PlayJointTrajectory)

request = PlayJointTrajectoryRequest()

joint_position = [JointAngle() for i in range(7)]
# joint_position = JointAngles[]
for i, j in enumerate(joint_position):
    j.joint_identifier = i + 1

msg = rospy.wait_for_message("/planned_trajectory", JointTrajectory)

for i in range(7): 
    if msg.points[0].positions[i]<0 :
        joint_position[i].value = msg.points[0].positions[i]*180.0/3.141592653 + 360
    
    else :
        joint_position[i].value = msg.points[0].positions[i]*180.0/3.141592653
    

    print(joint_position[i])

# extra = JointAngle()
# extra.joint_identifier = 1
# extra.value = 354.04
# joint_position.append(extra)

# jp2 = copy.deepcopy(joint_position)
# jp2[0].value = 354.04
# jp2[1].value = 9.88
# jp2[2].value = 165.87
# jp2[3].value = 277.08
# jp2[4].value = 4.28
# jp2[5].value = 276.02
# jp2[6].value = 61.73

# print(type(request))
request.input.joint_angles.joint_angles = joint_position


result = client(request)
print(result)
rospy.spin()