#!/usr/bin/python
import subprocess
import os
from threading import Thread

def bash_command(cmd):
    subprocess.Popen(['xterm', '-e', cmd], stdout=subprocess.PIPE,stdin=subprocess.PIPE, stderr=subprocess.PIPE)

if __name__ == "__main__":
    bash_command("rosrun follow_me decision_node; sleep 5000")
    bash_command("rosrun follow_me robot_moving_node; sleep 5000")
    bash_command("rosrun follow_me moving_persons_detector_node; sleep 5000")
    bash_command("rosrun follow_me group_detection_node; sleep 5000")
    bash_command("rosrun follow_me obstacle_detection_node; sleep 5000")
    if(len(sys.argv)>=3 and sys.argv[2] == "enable_dep"):
        bash_command("rosrun follow_me rotation_action_node; sleep 5000")
        bash_command("rosrun follow_me translation_action_node; sleep 5000")
    else:
        bash_command("rosrun teleoperation teleoperation_node.py; sleep 5000")
