#!/usr/bin/env python

import ConfigParser
import ipfshttpclient
import os
import rospy
import subprocess
import threading
import time

from geometry_msgs.msg import Twist
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64


class Robot(threading.Thread):
    def __init__(self):

        rospy.init_node('sample_controller', anonymous=False)
        rospy.loginfo("Node initialized")

        rospy.loginfo("Parsing Config")
        dirname = os.path.dirname(__file__) + '/../'
        configParser = ConfigParser.RawConfigParser()
        configFilePath = dirname + 'src/config.config'
        configParser.read(configFilePath)
        rospy.loginfo("Parsing completed")

        rospy.loginfo('Waiting for job payment, press Ctrl+\ to interrupt')

        program =  configParser.get('keys_and_addresses', 'ROBONOMICS_DIR') + "/robonomics io read launch"
        process = subprocess.Popen(program, shell=True, stdout=subprocess.PIPE)
        while True:
            try:
                output = process.stdout.readline()
                if output.strip() == configParser.get('keys_and_addresses', 'EMPLOYER_ADDRESS') + " >> " + configParser.get('keys_and_addresses', 'CURIOSITY_ADDRESS') + " : true":
                    rospy.loginfo("Job Paid!")
                    process.kill()
                    break
                if output.strip():
                    rospy.loginfo("Not my job is paid!")
            except KeyboardInterrupt:
                exit()

        rospy.loginfo("Start Working")
        self.stop_publishing_arming = False
        self.stop_publishing_moving = False
        self.stop_reading_state = False


        self.states = threading.Thread(target=self.listener)
        self.arm = threading.Thread(target=self.raise_up)
        self.move = threading.Thread(target=self.move)

        rospy.loginfo("Start collecting state")
        self.states.start()

        rospy.loginfo("Arming...")
        self.arm.start()
        ## TODO: start moving when arm and mast are set up
        time.sleep(25)
        self.stop_publishing_arming = True
        rospy.loginfo("Armed")
        self.arm.join()

        rospy.loginfo("Moving")
        self.move.start()
        time.sleep(60)
        self.stop_publishing_moving = True
        rospy.loginfo("Stopped moving")
        self.move.join()


        self.stop_reading_state = True
        self.states.join()
        rospy.loginfo("Stopped collecting state")

        rospy.loginfo("Pushing states to file")

        try:
            f = open(dirname + '/file_states.txt', 'w')
            for item in self.state:
                    f.write("%s\n" % item)
        finally:
            f.close()

        rospy.loginfo("Pushing file to IPFS")
        client = ipfshttpclient.connect()
        res = client.add(dirname + '/file_states.txt')
        rospy.loginfo("Pushed, the IPFS hash is " + res.values()[0].encode('utf8'))

        rospy.loginfo("Publishing IPFS hash to chain")
        program = "echo \"" + res.values()[0].encode('utf8') + "\" | " + configParser.get('keys_and_addresses', 'ROBONOMICS_DIR') + "/robonomics io write datalog -s " + configParser.get('keys_and_addresses', 'CURIOSITY_KEY')
        process = subprocess.Popen(program, shell=True, stdout=subprocess.PIPE)
        output = process.stdout.readline()
        rospy.loginfo("Published to chain! Transaction hash is " + output.strip())
        rospy.loginfo("Job Done. Check DAPP for IPFS data hash")


    def callback_wheel_state(self, data):
        self.state.append(data)

    # def callback_mast_p_state(self, data):
    #     rospy.loginfo(data)
    #
    # def callback_mast_02_state(self, data):
    #     rospy.loginfo(data)
    #
    # def callback_arm_01_state(self, data):
    #     rospy.loginfo(data)
    #
    # def callback_arm_02_state(self, data):
    #     rospy.loginfo(data)
    #
    # def callback_arm_03_state(self, data):
    #     rospy.loginfo(data)


    def listener(self):

        self.state = []
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rospy.Subscriber("curiosity_mars_rover/middle_wheel_L_joint_velocity_controller/state", JointControllerState, self.callback_wheel_state)

            # rospy.Subscriber("curiosity_mars_rover/mast_p_joint_position_controller/state", JointControllerState, self.callback_mast_p_state)
            # rospy.Subscriber("curiosity_mars_rover/mast_02_joint_position_controller/state", JointControllerState, self.callback_mast_02_state)
            #
            # rospy.Subscriber("curiosity_mars_rover/arm_01_joint_position_controller/state", JointControllerState, self.callback_arm_01_state)
            # rospy.Subscriber("curiosity_mars_rover/arm_02_joint_position_controller/state", JointControllerState, self.callback_arm_02_state)
            # rospy.Subscriber("curiosity_mars_rover/arm_03_joint_position_controller/state", JointControllerState, self.callback_arm_03_state)

            if self.stop_reading_state:
                break
            rate.sleep()




    def raise_up(self):

        mast1 = rospy.Publisher('curiosity_mars_rover/mast_p_joint_position_controller/command', Float64, queue_size=10)
        mast2 = rospy.Publisher('curiosity_mars_rover/mast_02_joint_position_controller/command', Float64, queue_size=10)

        arm1 = rospy.Publisher('curiosity_mars_rover/arm_01_joint_position_controller/command', Float64, queue_size=10)
        arm2 = rospy.Publisher('curiosity_mars_rover/arm_02_joint_position_controller/command', Float64, queue_size=10)
        arm3 = rospy.Publisher('curiosity_mars_rover/arm_03_joint_position_controller/command', Float64, queue_size=10)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            mast1.publish(0.0)
            mast2.publish(0.0)

            arm1.publish(0.2)
            arm2.publish(0.0)
            arm3.publish(0.0)
            if self.stop_publishing_arming:
                break
            rate.sleep()


    def move(self):
        move = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        circle_command = Twist()
        circle_command.linear.x = 10.0
        circle_command.linear.y = 0.0
        circle_command.linear.z = 0.0

        circle_command.angular.x = 0.0
        circle_command.angular.y = 0.0
        circle_command.angular.z = 0.0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            move.publish(circle_command)
            if self.stop_publishing_moving:
                circle_command.linear.x = 0.0
                circle_command.angular.z = 0.0
                move.publish(circle_command)
                break
            rate.sleep()


if __name__ == '__main__':
    robot = Robot()
