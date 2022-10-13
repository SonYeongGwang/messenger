#!/home/vision/.virtualenvs/torch_env/bin/python

import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose
from messenger.msg import MultiPoseInfo, SinglePoseInfo

import numpy as np
import yaml
import glob
import os
import time

def vision_start_callback(data):
    global vision_command
    global home_path

    rospy.loginfo("Vision Start Command: {}".format(data.data))
    vision_command = data.data
    if vision_command == 1:
        with open(home_path+'/catkin_ws/start_command.yaml', 'w') as f:
            yaml.dump(vision_command, f, default_flow_style=None)
            
        vision_command = 0

home_path = os.path.expanduser('~')
pub = rospy.Publisher('/object_meta_info', MultiPoseInfo, queue_size=10)
rospy.Subscriber('/vision_start_command', Int16, vision_start_callback)
rospy.init_node('messenger', anonymous=True)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    res = glob.glob(home_path+'/catkin_ws/pose*')
    if len(res) > 0:

        time.sleep(0.01)
        m_pose_info = MultiPoseInfo()
        pose_set = []

        with open(res[0]) as f:
            pose_info = yaml.load(f, Loader=yaml.FullLoader)
        # print(pose_info)
        num_of_objects = len(pose_info.keys())

        for i in range(num_of_objects):
            s_pose_info = SinglePoseInfo()
            s_pose_info.shape = pose_info[str(i)]['shape']
            s_pose_info.position.x = pose_info[str(i)]['position'][0]
            s_pose_info.position.y = pose_info[str(i)]['position'][1]
            s_pose_info.position.z = pose_info[str(i)]['position'][2]
            s_pose_info.orientation.x = pose_info[str(i)]['orientation'][0]
            s_pose_info.orientation.y = pose_info[str(i)]['orientation'][1]
            s_pose_info.orientation.z = pose_info[str(i)]['orientation'][2]
            s_pose_info.orientation.w = pose_info[str(i)]['orientation'][3]
            s_pose_info.shape_parameters = pose_info[str(i)]['shape_para']

            pose_set.append(s_pose_info)

        m_pose_info.meta_info = pose_set

        rospy.loginfo(m_pose_info)
        pub.publish(m_pose_info)

        os.system('rm '+res[0])

        rate.sleep()
