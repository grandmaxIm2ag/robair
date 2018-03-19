#!/usr/bin/python
import subprocess
import os
from threading import Thread

#p = subprocess.Popen("rosrun follow_me decision_node", stdout=subprocess.PIPE,stdin=subprocess.PIPE, shell=True)
#p = subprocess.Popen("rosrun follow_me robot_moving_node", stdout=subprocess.PIPE,stdin=subprocess.PIPE, shell=True)
#p = subprocess.Popen("rosrun follow_me moving_persons_detector_node", stdout=subprocess.PIPE,stdin=subprocess.PIPE, shell=True)
#p = subprocess.Popen("rosrun follow_me rotation_action_node", stdout=subprocess.PIPE,stdin=subprocess.PIPE, shell=True)
#p = subprocess.Popen("rosrun follow_me obstacle_detection_node", stdout=subprocess.PIPE,stdin=subprocess.PIPE, shell=True)
#p = subprocess.Popen("rosrun follow_me translation_action_node", stdout=subprocess.PIPE,stdin=subprocess.PIPE, shell=True)

def bash_command(cmd):
    subprocess.Popen(['xterm', '-e', cmd], stdout=subprocess.PIPE,stdin=subprocess.PIPE, stderr=subprocess.PIPE)

bash_command("rosrun follow_me decision_node; sleep 5000")
bash_command("rosrun follow_me robot_moving_node; sleep 5000")
bash_command("rosrun follow_me moving_persons_detector_node; sleep 5000")
bash_command("rosrun follow_me rotation_action_node; sleep 5000")
bash_command("rosrun follow_me obstacle_detection_node; sleep 5000")
bash_command("rosrun follow_me translation_action_node; sleep 5000")
bash_command("rosrun follow_me group_detection_node; sleep 5000")
