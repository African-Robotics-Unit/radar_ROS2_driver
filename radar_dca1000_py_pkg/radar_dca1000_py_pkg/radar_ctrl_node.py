#! /usr/bin/env python3
# Standard Python Imports
import socket
import struct 
import time

# M2S2 imports
from m2s2_pydevclass.m2s2_device import M2S2Device

# Package Submodules
from .submodules.mmwave_config_class import mmWaveConfig

# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# ROS2 Interface imports 
from m2s2_interfaces.srv import SendDeviceControlCommand
from m2s2_interfaces.msg import ControlStatus

class MMWaveControlNode(Node):
    # %---------------------------------------------------------------------------------------------------------
    # Class Parameters
    # %---------------------------------------------------------------------------------------------------------
    # useful commands
    dev_cpt_cmd = [ 
        'sensorStop\n',
        'sensorStart\n'
        ] 

    dca_cfg_cmd = { 
        'RESET_FPGA_CMD_CODE'               : b"\x5a\xa5\x01\x00\x00\x00\xaa\xee", 
        'CONFIG_FPGA_GEN_CMD_CODE'          : b"\x5a\xa5\x03\x00\x06\x00\x01\x02\x01\x02\x03\x1e\xaa\xee", 
        'CONFIG_PACKET_DATA_CMD_CODE'       : b"\x5a\xa5\x0b\x00\x06\x00\xbe\x05\x35\x0c\x00\x00\xaa\xee", 
        'SYSTEM_CONNECT_CMD_CODE'           : b"\x5a\xa5\x09\x00\x00\x00\xaa\xee", 
        'RECORD_START_CMD_CODE'             : b"\x5a\xa5\x05\x00\x00\x00\xaa\xee", 
        'SYSTEM_ERROR_CMD_CODE'             : b"\x5a\xa5\x0a\x00\x01\x00\xaa\xee", 
        'RECORD_STOP_CMD_CODE'              : b"\x5a\xa5\x06\x00\x00\x00\xaa\xee", 
        }   
        # commands sent over ethernet based on CLI. Consists of header, data size, command code, footer
        # commands above are listed in order of send and receive
        # some commands in the DCA1000 user guide are not used and are thus not included
    
    dev_err_codes = { 
        "-50"      : "Unknown command", 
        "-51"      : "Invalid usage of command", 
        "-52"      : "Invalid input parameter in command", 
        "-53"      : "Sensor start reconfiguration not supported", 
        "-54"      : "Sensor has already been started or stopped", 
        "-55"      : "LVDS Software Session or HSI header not supported", 
        "-56"      : "Application execution timeout", 
        "-57"      : "Error in datapath configuration", 
        "-58"      : "Command not supported while frame in progress", 
        }  

    # M2S2 Device Specification 
    device = None

    # Sockets and Ports
    dca_cmd_addr = ('192.168.33.180',4096)      # address to send commands to
    dca_rcv_addr = ('192.168.33.30', 4096)      # address to receive command responses from
    ser_cmd_addr = ('137.158.125.136', 8080)    # address of the serial server device
    dca_socket = None                           # socket variable for the command port
    ser_socket = None                           # socket variable for the serial command port

    # configs
    runDict = None # runtime parameter dictionary 
    devDict = None # device parameter dictionary

    # flags
    errDCA_ = None  # True -> error configuring DCA1000
    cfgDCA_ = None  # True -> DCA successfully configured
    errDEV_ = None  # True -> error configuring Device
    cfgDEV_ = None  # True -> Radar device successfully configured
      

    # %---------------------------------------------------------------------------------------------------------
    # Class Functions
    # %---------------------------------------------------------------------------------------------------------
    # Class constructor
    def __init__(self):

        ## Start Node
        self.device = M2S2Device("RADAR")
        super().__init__("%s_node" % self.device.devNAME) 

        ## Setup Radar System Components -----------------------------------------------------------------------
        # set flags to defualt
        self.errDCA_ = False
        self.cfgDCA_ = False
        self.errDEV_ = False
        self.cfgDEV_ = False

        # store file to config JSONs path in ROS2 parameters
        self.declare_parameter('run_cfg_file_path', '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgRun.json')
        self.declare_parameter('dev_cfg_file_path', '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgDev.json')
        self.runConfigPath = self.get_parameter('run_cfg_file_path').get_parameter_value().string_value
        self.devConfigPath = self.get_parameter('dev_cfg_file_path').get_parameter_value().string_value

        # create socket for command port (all radar commands and responses sent and collected here)
        try:
            self.dca_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.dca_socket.bind(self.dca_rcv_addr)
            self.dca_socket.settimeout(1)
            self.get_logger().info("DCA1000 Command Socket Created Successfully") 
        except Exception as e:
            self.errDCA_ = True
            self.get_logger().info("DCA1000 Command Socket Creation Failed With Exception: %s" % (str(e)))


        ## Setup ROS2 Components -------------------------------------------------------------------------------
        # create callback groups
        self.cllgrpTIMER = MutuallyExclusiveCallbackGroup()
        self.cllgrpSERVR = MutuallyExclusiveCallbackGroup()

        # create timer and client
        self.pubTIMER = self.create_timer(1,self.publish_Status,self.cllgrpTIMER)
        self.staPBLSH = self.create_publisher(ControlStatus,self.device.devSPUB,10)

        # create radar control service
        self.radSERVR = self.create_service(SendDeviceControlCommand, 
                                            self.device.devSERV, 
                                            self.radar_Service)
                                            #callback_group=self.cllgrpSERVER)

        # finish node init
        self.get_logger().info("%s Control Node Started" % self.device.devNAME)
        self.device.devFLAGS_['PWRON'] = True

    # Primary Functions
    # %---------------------------------------------------------------------------------------------------------
    def radar_Service(self, request, response):
        """Provides interaction with node from main user control node. Request can be SETUP, START or STOP.  
    For setup -> Configures DCA1000 over ethernet and then configures the radar via ROS2 service.
    For start -> First sends the record start command over ethernet to tell the DCA1000 to start monitoring LVDS lanes.
    For stop  -> First sends the sensorStop command over serial to tell the radar to stop transmitting.
    
    :param int toggle: toggle is either a 1 or 0. 1 starts recording and 0 stops recording
    :raises AnyError: raises error and exits program execution if error response was received from startSensor or startRecord command
    """

        ## Configure Radar System --------------------------------------------------------------------------------
        if (request.command == request.SETUP):
            self.cfgDEV_ = False
            self.cfgDCA_ = False
            self.get_logger().info("Received SETUP Request from Client. Loading Configs.")
            self.configs = mmWaveConfig(self.runConfigPath,self.devConfigPath)
            self.runDict = self.configs.runtimeParamDict
            self.devDict = self.configs.mmwaveParamDict
            if not self.device.devFLAGS_["RECRD"]:
                self.setup_DCA1000()
                self.setup_RADAR() 
            else:
                self.get_logger().info("Device is running. Stop device before attempting config.\n")

            self.device.devFLAGS_["SETUP"] = self.cfgDCA_ and self.cfgDEV_
            self.display_Status()
            response.status = (self.device.devFLAGS_["SETUP"])
            return response
        else:

            if (request.command == request.START):
                self.get_logger().info("Received START Request from Client.")
                self.dca_socket.sendto(self.dca_cfg_cmd['RECORD_START_CMD_CODE'], self.dca_cmd_addr)
                self.response_DCA1000('RECORD_START_CMD_CODE')
        
                attempts = 0
                tcp_response = None
                while tcp_response == None:
                    tcp_response = self.send_Request(self.dev_cpt_cmd[request.START])
                    attempts += 1
                    if attempts >= 5:
                        self.errDEV_ = True
                        break
                if tcp_response == "Done" and not self.errDCA_:
                    self.device.devFLAGS_["RECRD"] = True
                    self.get_logger().info("Radar has started recording. Waiting for stop command.\n")
                    self.display_Status()
                    response.status = self.device.devFLAGS_["RECRD"]
                    return response
                else:
                    for err in self.dev_err_codes:
                        if err in tcp_response:
                            self.get_logger().error(self.dev_err_codes[err] + "\n")
                    self.errDEV_ = True

            elif (request.command == request.STOP):     
                self.get_logger().info("Received STOP Request from Client.")   
                
                attempts = 0
                tcp_response = None
                while tcp_response == None:
                    tcp_response = self.send_Request(self.dev_cpt_cmd[request.STOP])
                    attempts += 1
                    if attempts >= 5:
                        self.errDEV_ = True
                        break
                
                if tcp_response == "Done":
                    self.device.devFLAGS_["RECRD"] = False
                    self.dca_socket.sendto(self.dca_cfg_cmd['RECORD_STOP_CMD_CODE'], self.dca_cmd_addr)
                    self.response_DCA1000('RECORD_STOP_CMD_CODE')
                    self.get_logger().info("Radar has stopped recording. Ready for next record session.\n")
                    self.display_Status() 
                    response.status = True
                    return response
                else:
                    for err in self.dev_err_codes:
                        if err in tcp_response:
                            self.get_logger().error(self.dev_err_codes[err] + "\n")
                    self.errDEV_ = True
            
            self.display_Status()
            response.status = (not self.errDEV_) and (not self.errDCA_)
            return response

    def publish_Status(self):
        msg = ControlStatus()
        msg.dev_name = self.device.devNAME             # device name
        msg.pwron = self.device.devFLAGS_["PWRON"]     # device node is running 
        msg.setup = self.device.devFLAGS_["SETUP"]     # device is setup
        msg.recrd = self.device.devFLAGS_["RECRD"]     # device is recording and publishing
        self.staPBLSH.publish(msg)
        #self.display_Status()

    def display_Status(self):
        self.get_logger().info("%s Status: PWRON: %s, SETUP: %s, RECRD %s.\n" %
                                    (self.device.devNAME,              
                                    self.device.devFLAGS_["PWRON"],      
                                    self.device.devFLAGS_["SETUP"],     
                                    self.device.devFLAGS_["RECRD"]))


    # Helper Functions
    # %---------------------------------------------------------------------------------------------------------    
    def setup_DCA1000(self):
        """Performs setup of DCA1000 over etherent. 
    Sends commands over etherent to command port of DCA1000. Commands
    are listed in dictionary at start of class definition. 
    Function Can cause program to hang on waiting for successful acknowledge of 
    command from DCA1000. 
    :raises AnyError: raises errors if setup commands failed
    """
        if self.errDCA_:
            self.get_logger().error("DCA1000 setup failed: Socket not open.")
            return

        # Set up DCA
        self.get_logger().info("Starting DCA1000 Setup.\n")

        # Setup Command List (Ordered List)
        cfgCmdList = [
            'RESET_FPGA_CMD_CODE',
            'CONFIG_FPGA_GEN_CMD_CODE',
            'CONFIG_PACKET_DATA_CMD_CODE',
            'SYSTEM_CONNECT_CMD_CODE'
            ]

        for cmd in cfgCmdList:
            self.dca_socket.sendto(self.dca_cfg_cmd[cmd], self.dca_cmd_addr)
            self.response_DCA1000(cmd)

        if self.errDCA_:
            self.get_logger().error("DCA1000 setup failed: One or more config commands failed.")
        else:
            self.cfgDCA_ = True
    
    # %---------------------------------------------------------------------------------------------------------    
    def setup_RADAR(self):
        """Performs setup of radar using a ROS2 service. 
    This ROS node was developed for use with the Nvidia Jetson Orin. The Orin has a driver
    issue that prevents usb communications with the radar. As such the Orin needs to send the
    commands to another computer of network using tcp protocol. This function calls a ROS2
    TCP service to send strings to the external computer (server).
    :raises AnyError: raises errors if setup commands failed
    """
        # Set up Radar Device
        variant = self.runDict["VARIANT"]
        self.get_logger().info("Starting %s Radar Device Setup." % (variant))

        for cmdKey in list(self.devDict.keys()):
            
            cmdString = self.configs.get_mmWaveCommandString(cmdKey)
            attempts = 0
            tcp_response = None
            while tcp_response == None:
                #self.get_logger().info("Sending %s" % (cmdString))
                tcp_response = self.send_Request(cmdString)
                attempts += 1
                if attempts >= 5:
                    self.errDEV_ = True
                    break

            if tcp_response == "Done":
                pass
            else:
                for err in self.dev_err_codes:
                    if err in tcp_response:
                        self.get_logger().error(self.dev_err_codes[err])
                    self.errDEV_ = True
            
        if self.errDEV_:
            self.get_logger().error("Radar Device setup failed: One or more config commands failed.")
        else:
            self.get_logger().info("Radar Device setup succeeded.\n") 
            self.cfgDEV_ = True

    # %---------------------------------------------------------------------------------------------------------
    def response_DCA1000(self, cmd):
        """ Helper function to listen for given command response from DCA1000. 
        Will wait for command response. Can cause program to hang if acknowledgement packet is missed. 
        Sets the DCA1000 error flag to true if status is 1. 
        :param string cmd: Name of the command sent to DCA1000 and waiting for response. 
        """
        self.get_logger().info("Waiting for " + cmd)

        received = False
        timeoutCount = 0
        status=1
        while (not received) and (timeoutCount<=5):
            try:
                msg, server = self.dca_socket.recvfrom(2048)
                (status,) = struct.unpack('<H', msg[4:6])
                received = True
            except Exception as e:
                self.get_logger().error(cmd + " Failed With Exception: " + str(e))
                timeoutCount += 1
                continue

        if status == 0:
            self.get_logger().info(cmd + " was successful.\n")
        else:
            self.get_logger().error(cmd + " was unsuccessful.")
            self.get_logger().error("Received Following Status: " + str(status) + "\n")
            self.errDCA_ = True

    # %---------------------------------------------------------------------------------------------------------
    def response_SerialServer(self, cmd):
        """ Helper function to listen for given command response from Radar serial server being used. 
        Will wait for command response. Can cause program to hang if acknowledgement packet is missed. 

        :param string cmd: Name of the command sent to DCA1000 and waiting for response. 
        """
        received = False
        timeoutCount = 0
        status=""
        while (not received) and (timeoutCount<=5):
            try:
                msg, server = self.ser_socket.recvfrom(2048)
                # print(msg.decode('utf-8')) # uncomment to get exact response received for
                received = True
            except Exception as e:
                self.get_logger().error(cmd + " Failed With Exception: " + str(e))
                timeoutCount += 1
        try:
            status = msg.decode('utf-8','ignore').split("\n")[1]
        except:
            if not len(msg) == 0:
                status = "Received inconsisten message from server: " + msg.decode('utf-8')
            else: 
                status = "Nothing received from server."


        return status

    # %---------------------------------------------------------------------------------------------------------
    def send_Request(self, cmdString):
        time.sleep(0.02)
        self.get_logger().info("Sending \"%s\"" % (cmdString.strip('\n')))
        self.ser_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ser_socket.settimeout(1)
        try:
            self.ser_socket.connect(self.ser_cmd_addr)
            self.ser_socket.sendto(bytes(cmdString,'utf-8'), self.ser_cmd_addr)
        except Exception as e:
            self.errDEV_ = True
            self.get_logger().info("Connection failed with exception: %s" % (str(e)))  
            tcp_response = "No connection to server."

        try:
            tcp_response = self.response_SerialServer(cmdString.strip('\n'))
        except Exception as e:
            self.errDEV_ = True
            self.get_logger().info("Response failed with exception: %s" % (str(e)))  
            tcp_response = "Issue with response format."
        self.ser_socket.close()


        if tcp_response == "Done":
            self.get_logger().info('Received Status: \"%s\"\n' % (tcp_response))
        else:
            self.get_logger().info('Received Status: \"%s\"' % (tcp_response))
            self.errDEV_ = True
        return tcp_response

    # %---------------------------------------------------------------------------------------------------------
    # END OF CLASS
    # %---------------------------------------------------------------------------------------------------------        

def main(args=None):
    rclpy.init()
    node = MMWaveControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info("Press Ctrl+C to shutdown.\n")
        executor.spin()
        #rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.\n")
        node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()