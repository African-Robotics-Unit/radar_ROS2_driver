#! /usr/bin/env python3
# Standard Python Imports
import time
import sys

# M2S2 imports
from m2s2_pydevclass.m2s2_device import M2S2Device

# ROS2 Imports
import rclpy
from rclpy.node import Node

# ROS2 Interface imports 
from m2s2_interfaces.srv import SendDeviceControlCommand


class MMWaveClientNode(Node):
    # %---------------------------------------------------------------------------------------------------------
    # Class Parameters
    # %---------------------------------------------------------------------------------------------------------
    
    # M2S2 Device Specification 
    device = None

    def __init__(self):

        ## Start Node
        self.device = M2S2Device("RADAR")
        super().__init__("%s_client_node" % self.device.devNAME) 

        self.cli = self.create_client(SendDeviceControlCommand, self.device.devSERV)
        #print("dca1000/" + self.device.devSERV)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SendDeviceControlCommand.Request()

    def send_request(self, cmd):
        self.req.command = cmd
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    

def main(args=None):
    rclpy.init(args=args)

    cmdDICT = {
        2:"SETUP",
        1:"START",
        0:"STOP"
    }
    try:
        request = str(sys.argv[1])
    except:
        request = None
    if request == "-h" or request == "--help" or request == None:
        print("Command List:")
        print("SETUP -> Setup up the radar device (required for first time)")
        print("START -> Start Recording")
        print("STOP  -> Stop Recording")
    else:
        print("Request: %s" % request)
        requestNum = None
        for cmdNum in cmdDICT:
            if request == cmdDICT[cmdNum]:
                requestNum = cmdNum
        if requestNum == None:
            print("Invalid Command")
            print("Run with -h or --help to view available commands")
        else:
            node = MMWaveClientNode()
            response = node.send_request(requestNum)
            print(response)
            node.destroy_node()
    rclpy.shutdown()
