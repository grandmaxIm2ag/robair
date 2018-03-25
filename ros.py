#!/usr/bin/python
import subprocess
import sys
import os
from threading import Thread
from optparse import OptionParser

def bash_command(cmd):
    subprocess.Popen(['xterm', '-e', cmd], stdout=subprocess.PIPE,stdin=subprocess.PIPE, stderr=subprocess.PIPE)

def roscore(bag_data):
    bash_command("roscore; sleep 5000")
    bash_command("rosbag play bag_data; sleep 5000")

def teleoperation():
    bash_command("rosrun follow_me obstacle_detection_node; sleep 5000")
    bash_command("rosrun teleoperation teleoperation_node.py; sleep 5000")

def distribution(b):
    if(b):
        bash_command("rviz; sleep 5000")
    bash_command("rosrun follow_me decision_node; sleep 5000")
    bash_command("rosrun follow_me robot_moving_node; sleep 5000")
    bash_command("rosrun follow_me moving_persons_detector_node; sleep 5000")
    bash_command("rosrun follow_me group_detection_node; sleep 5000")
    
def main():
    parser = OptionParser(usage="usage: %prog [options] filename",
                          version="%prog 1.0")

    parser.add_option("-r", "--rviz",action="store_true",dest="rviz",
                      default=False)
    parser.add_option("-e", "--enable_dep",action="store_true", dest="enable_dep",
                      default=True)
    parser.add_option("-d", "--disable_dep",action="store_true", dest="disable_dep",
                      default=False)
    parser.add_option("--roscore", "--roscore", dest="filename",
                      default=False)
    (options, args) = parser.parse_args()

    print options
    
    distribution(options["rviz"])
    if(options["enable_dep"]):
        print 'ca bouge'
    if(options["disable_dep"]):
        teleoperation()
    if(options["roscore"]):
        roscore(options["filename"])
    
    
   
if __name__ == "__main__":
    main()
