#!/usr/bin/env python3

import ipfshttpclient
import rospy
import threading
import typing as tp
import yaml

from geometry_msgs.msg import Twist
from control_msgs.msg import JointControllerState
from os import path, remove
from std_msgs.msg import Float64
from substrateinterface import SubstrateInterface, Keypair
from time import sleep

'''
Robonomics functions and class for working with substrate
'''


def read_yaml_file(yaml_path: str) -> tp.Dict or None:
    """
    Read a yaml file
    Parameters
    ----------
    yaml_path : path to yaml file
    Returns
    -------
    yaml file contains as dictionary
    """
    rospy.loginfo(f"Reading .yaml file {yaml_path}")
    if not path.exists(yaml_path):
        rospy.logerr(f"{yaml_path} not found")
        return None

    with open(yaml_path, "r") as file:
        try:
            dictionary = yaml.safe_load(file)
            return dictionary
        except Exception as Err:
            rospy.logerr(f"Error loading {yaml_path}: {Err}")
            return None


def substrate_connection(url: str) -> tp.Any:
    """
    establish connection to a specified substrate node
    """
    try:

        rospy.loginfo("Establishing connection to substrate node")
        substrate = SubstrateInterface(
            url=url,
            ss58_format=32,
            type_registry_preset="substrate-node-template",
            type_registry={
                "types": {
                    "Record": "Vec<u8>",
                    "Parameter": "Bool",
                    "LaunchParameter": "Bool",
                    "<T as frame_system::Config>::AccountId": "AccountId",
                    "RingBufferItem": {
                        "type": "struct",
                        "type_mapping": [
                            ["timestamp", "Compact<u64>"],
                            ["payload", "Vec<u8>"],
                        ],
                    },
                    "RingBufferIndex": {
                        "type": "struct",
                        "type_mapping": [
                            ["start", "Compact<u64>"],
                            ["end", "Compact<u64>"],
                        ],
                    }
                }
            },
        )
        rospy.loginfo("Successfully established connection to substrate node")
        return substrate
    except Exception as e:
        rospy.logerr(f"Failed to connect to substrate: {e}")
        return None


def write_datalog(substrate, seed: str, data: str) -> str or None:
    """
    Write any string to datalog

    Parameters
    ----------
    substrate : substrate connection instance
    seed : mnemonic seed of account which writes datalog
    data : data tp be stored as datalog

    Returns
    -------
    Hash of the datalog transaction
    """

    # create keypair
    try:
        keypair = Keypair.create_from_mnemonic(seed, ss58_format=32)
    except Exception as e:
        rospy.logerr(f"Failed to create keypair for recording datalog: \n{e}")
        return None

    try:
        rospy.loginfo("Creating substrate call for recording datalog")
        call = substrate.compose_call(
            call_module="Datalog",
            call_function="record",
            call_params={
                'record': data
            }
        )
        rospy.loginfo(f"Successfully created a call for recording datalog:\n{call}")
        rospy.loginfo("Creating extrinsic for recording datalog")
        extrinsic = substrate.create_signed_extrinsic(call=call, keypair=keypair)
    except Exception as e:
        rospy.logerr(f"Failed to create an extrinsic for recording datalog: {e}")
        return None

    try:
        rospy.loginfo("Submitting extrinsic for recording datalog")
        receipt = substrate.submit_extrinsic(extrinsic, wait_for_inclusion=True)
        rospy.loginfo(f"Extrinsic {receipt.extrinsic_hash} for recording datalog sent and included in block"
                      f" {receipt.extrinsic_hash}")
        return receipt.extrinsic_hash
    except Exception as e:
        rospy.logerr(f"Failed to submit extrinsic for recording datalog: {e}")
        return None


def pin_file_in_ipfs(filepath: str, remove_after=True) -> str or None:
    """
    push file to IPFS. Then remove the file

    Parameters
    ----------
    remove_after : remove file after pinning to save space or not
    filepath : path to file to be pinned

    Returns
    -------
    IPFS hash of a pinned file
    """

    try:
        rospy.loginfo(f"Pushing file {filepath} to IPFS")
        ipfs_client = ipfshttpclient.connect()
        res = ipfs_client.add(filepath)
        hash = res["Hash"]
        ipfs_client.close()
        rospy.loginfo(f"File pushed to IPFS. Hash is {hash}")
    except Exception as e:
        rospy.logerr(f"Failed to push file to local IPFS node. Error: {e}")
        hash = None
        try:
            ipfs_client.close()
        except Exception:
            pass

    if remove_after:
        try:
            remove(filepath)
            rospy.loginfo("File removed")
        except Exception as e:
            rospy.logerr(f"Failed to remove file: {e}")

    return hash


class LaunchTracker:
    """
    Parse new blocks in chain to search for Launch commands
    """

    def __init__(self, substrate: tp.Any, source_address: str, target_address: str):
        """
        Starting subscriber and creating Python event

        :param substrate: instance of substrate connection
        :param source_address: address from which tre launch transaction is supposed
        :param target_address: robot address
        """
        rospy.loginfo("Creating an instance of a LaunchTracker class")
        self.substrate = substrate
        self.employer_address = source_address
        self.robot_address: str = target_address

        rospy.loginfo(f"Initiating new blocks subscriber for launch commands tracking")
        self.launch_command_event = threading.Event()
        self.subscriber = threading.Thread(target=self._obtain_launch_commands)
        self.subscriber.start()

        rospy.loginfo("Block subscriber started. Waiting for launch commands")

    def _subscription_handler(self, obj, update_nr, subscription_id):
        """
        parse block events and trigger python Event on launch command sending to the robot address

        params info:
        https://github.com/polkascan/py-substrate-interface/blob/65f247f129016f5bb3a9a572579033f35fd385ea/substrateinterface/base.py#L2588
        """
        ch: str = self.substrate.get_chain_head()
        chain_events = self.substrate.get_events(ch)
        for ce in chain_events:
            if ce.value["event_id"] == "NewLaunch":
                print(ce.params)
            if ce.value["event_id"] == "NewLaunch" and ce.params[0]["value"] == self.employer_address \
                    and ce.params[1]["value"] == self.robot_address and ce.params[2]["value"] is True:  # yes/no
                rospy.loginfo(f"\"ON\" launch command from employer.")
                self.launch_command_event.set()  # trigger python Event in main loop

            elif ce.value["event_id"] == "NewLaunch" and ce.params[0]["value"] != self.employer_address \
                    and ce.params[1]["value"] == self.robot_address:
                rospy.loginfo(f"Launch command not from employer. Idle")

    def _obtain_launch_commands(self):
        """
        Subscribe to new block headers as soon as they are available. The callable `subscription_handler` will be
        executed when a new block is available and execution will block until `subscription_handler` will return
        a result other than `None`
        """

        self.substrate.subscribe_block_headers(self._subscription_handler)


'''
Robot class with methods to control the rover
'''


class Robot:

    def __init__(self):
        """
        State all constants and attributes, start launch command event waiting loop
        """
        self.state = []
        self.stop_reading_state = False
        self.states_thread = threading.Thread(target=self.listener)
        self.dirname = path.dirname(__file__) + '/../'

        rospy.init_node('sample_controller', anonymous=False)
        rospy.loginfo("Node initialized")

        rospy.loginfo("Parsing Config")
        dirname = path.dirname(__file__) + '/../'
        config = read_yaml_file(dirname + "src/config.yaml")

        self.curiosity_address = config['curiosity_address']
        self.curiosity_seed = config['curiosity_seed']
        self.employer_address = config['employer_address']
        self.node_address = config['node_address']
        rospy.loginfo("Parsing completed")

        rospy.loginfo('Initiating substrate connection for launch tracking and datalogs writing')

        self.substrate_launch = substrate_connection(self.node_address)
        self.substrate_datalog = substrate_connection(self.node_address)
        launch_tracker = LaunchTracker(self.substrate_launch, self.employer_address, self.curiosity_address)

        rospy.loginfo('Waiting job command from employer, press Ctrl+\\ to interrupt')

        while True:
            launch_tracker.launch_command_event.wait()
            self.work()
            launch_tracker.launch_command_event.clear()

    def work(self) -> bool:
        """
        Curiosity job function. Arm, collect states and move for 1 min

        :return: False if error or nothing is success
        """
        rospy.loginfo("Start Working")

        rospy.loginfo("Arm tools command")
        self.raise_up()

        rospy.loginfo("Collect state command")
        self.stop_reading_state = False
        self.states_thread.start()

        rospy.loginfo("Move command")
        self.move(True)

        sleep(60)
        rospy.loginfo("Stop movement command")
        self.move(False)

        rospy.loginfo("Stop collecting state command")
        self.stop_reading_state = True
        self.states_thread.join()

        rospy.loginfo("Pushing states to file")

        try:
            f = open(self.dirname + '/file_states.txt', 'w')
            for item in self.state:
                f.write("%s\n" % item)
            f.close()
            self.state = []
        except Exception as e:
            rospy.logerr(f"Failed to create states file: {e}")
            self.state = []
            return False

        rospy.loginfo("Pushing file to IPFS")
        hash = pin_file_in_ipfs(self.dirname + '/file_states.txt', remove_after=True)
        if not hash:
            return False

        rospy.loginfo("Publishing IPFS hash to chain")
        tr_hash = write_datalog(self.substrate_datalog, self.curiosity_seed, hash)
        if not tr_hash:
            return False

        rospy.loginfo("Published to chain! Transaction hash is " + tr_hash)
        rospy.loginfo("Job Done. Check DAPP for IPFS data hash")

    def callback_wheel_state(self, data: tp.Dict):
        """
        Collect telemetry from Curiosity

        :param data: States of a robot
        """
        self.state.append(data)

    def listener(self):
        """
        States listener function. Fills in self.state attribute
        """
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rospy.Subscriber("curiosity_mars_rover/middle_wheel_L_joint_velocity_controller/state",
                             JointControllerState, self.callback_wheel_state)
            if self.stop_reading_state:
                break
            rate.sleep()
        print('finished')

    @staticmethod
    def raise_up():
        """
        Arms mast and robot arm of Curiosity. Done by publishing joint states to the robot
        """
        mast1 = rospy.Publisher('curiosity_mars_rover/mast_p_joint_position_controller/command', Float64, queue_size=10)
        mast2 = rospy.Publisher('curiosity_mars_rover/mast_02_joint_position_controller/command', Float64,
                                queue_size=10)

        arm1 = rospy.Publisher('curiosity_mars_rover/arm_01_joint_position_controller/command', Float64, queue_size=10)
        arm2 = rospy.Publisher('curiosity_mars_rover/arm_02_joint_position_controller/command', Float64, queue_size=10)
        arm3 = rospy.Publisher('curiosity_mars_rover/arm_03_joint_position_controller/command', Float64, queue_size=10)

        sleep(2)
        mast1.publish(0.0)
        mast2.publish(0.0)

        arm1.publish(0.2)
        arm2.publish(0.0)
        arm3.publish(0.0)

    @staticmethod
    def move(start_stop: bool):
        """
        Make rover move by sending cmd_vel commands to controller

        :param start_stop: whether start or stop
        """
        move = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        circle_command = Twist()
        circle_command.linear.x = 10.0 if start_stop else 0.0
        circle_command.linear.y = 0.0
        circle_command.linear.z = 0.0

        circle_command.angular.x = 0.0
        circle_command.angular.y = 0.0
        circle_command.angular.z = 1.0 if start_stop else 0.0

        sleep(2)
        move.publish(circle_command)


if __name__ == '__main__':
    robot = Robot()
