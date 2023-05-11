#! /usr/bin/env python3
# Standard Python Imports
import socket
import struct 
import time
import os
import threading

# Python imports
from ringbuf import RingBuffer

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
from radar_interfaces.msg import Frame


class MMWaveDataNode(Node):
    # %---------------------------------------------------------------------------------------------------------
    # Class Parameters
    # %---------------------------------------------------------------------------------------------------------
    
    # M2S2 Device Specification 
    device = None

    # testing 
    softwareTest = False
    testPacket = b"".join([b"\x00\xff"]*int(1456/2))
    dataTest = False

    # Sockets and Ports
    dca_dat_addr = ('192.168.33.30', 4098)      # address to receive radar data from
    dat_socket = None                           # socket variable for the data stream port
    udpbuf_size = 65536                         # default size of UDP socket packet buffer

    # configs
    runDict = None # runtime parameter dictionary 
    devDict = None # device parameter dictionary

    # buffer
    frmSIZE = None  # size of frame in bytes 
    frmBUFF = None  # frame buffer (UDP packets will be stored in circular buffer) frmBUFF
    frmCNT  = None  # frame count
    bytWRT  = 0
    allFRMS = None
    pckNUM = None  # current packet number       

    def __init__(self):

        ## Start Node
        self.device = M2S2Device("RADAR")
        super().__init__("%s_data_node" % self.device.devNAME) 

        ## Setup Radar System Components ----------------------------------------------------------------------
        # store file to config JSONs path in ROS2 parameters
        self.declare_parameter('run_cfg_file_path', '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgRun.json')
        self.declare_parameter('dev_cfg_file_path', '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgDev.json')
        #self.declare_parameter('run_cfg_file_path', '/home/nedwob/ros2_ws/src/MyM2S2Code/radar_ROS2_driver/cfgRun.json')
        #self.declare_parameter('dev_cfg_file_path', '/home/nedwob/ros2_ws/src/MyM2S2Code/radar_ROS2_driver/cfgDev.json')
        runConfigPath = self.get_parameter('run_cfg_file_path').get_parameter_value().string_value
        devConfigPath = self.get_parameter('dev_cfg_file_path').get_parameter_value().string_value

        # create mmWaveConfig Object and store parameter dicionaries
        self.configs = mmWaveConfig(runConfigPath,devConfigPath)
        self.runDict = self.configs.runtimeParamDict
        self.devDict = self.configs.mmwaveParamDict

        # Create subdirectory for adc_data storage
        if self.dataTest:
            self.folderCount = 0
            mkdirSuccess = False
            while not mkdirSuccess:
                try:
                    os.mkdir("RadarDataTest_"+str(self.folderCount))
                    mkdirSuccess=True
                except Exception as e:
                    self.folderCount+=1 # if folder already present increment suffix by 1. "_0" -> "_1" etc.
            self.get_logger().info("Created new directory for data.") 
            self.fileName = "Frame_"

        # create socket for data port (all radar data traffic is collected from here)
        if not self.softwareTest:
            try:
                self.dat_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.dat_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,self.udpbuf_size*self.runDict['UDP_BUFFER_MLT'])
                self.dat_socket.bind(self.dca_dat_addr)
                self.dat_socket.settimeout(25e-5)
                self.get_logger().info("DCA1000 Data Socket Created Successfully\n") 
            except Exception as e:
                self.get_logger().info("DCA1000 Data Socket Creation Failed With Exception: %s\n" % (str(e)))

        # create frame buffer for data stream
        self.get_logger().info("Initializing Frame Buffer.")
        numChirps = self.devDict["frameCfg"]["numChirps"]
        numSamples = self.devDict["profileCfg"]["numAdcSamples"]
        bytesPerSample = 2 # adc either returns 12,14 or 16bit data which all have to be stored over 2 bytes 
        realOrComplex = (self.devDict["adcbufCfg"]["adcOutputFmt"]+2) % 3 # if 0 it is complex so 2%3 = 2, if 1 it is real 3%3=1
        recBitMask =  bin(self.devDict["channelCfg"]["rxChannelEn"])[2:] # convert number to bit mask string with bin() and remove 0b from start of string  
        numReceivers = 0
        for bit in recBitMask:  # count the number of bits = 1 in bit mask to get number of receiver channels enabled
            numReceivers = numReceivers + int(bit)
        self.frmSIZE = numChirps*numSamples*bytesPerSample*realOrComplex*numReceivers # calculate required buffer Size
        self.frmBUFF = RingBuffer(format='B', capacity=self.frmSIZE*self.runDict['FRAME_BUFFER_MLT'])
        # self.frmBUFF = frameBuffer(
        #                 frameSize=self.frmSIZE,
        #                 capMultiplier=self.runDict['FRAME_BUFFER_MLT'])

        self.get_logger().info("Frame size calculated as: %d Bytes" % (self.frmSIZE))        
        self.get_logger().info("Frame buffer size set to: %dx%d = %d Bytes\n" % 
                                    (self.frmSIZE, 
                                    self.runDict['FRAME_BUFFER_MLT'], 
                                    self.frmSIZE*self.runDict['FRAME_BUFFER_MLT']))

        self.frmCNT  = 0 
        self.pckNUM = 0   
        self.allFRMS = False

        ## Setup ROS2 Components -------------------------------------------------------------------------------
        # self.cllgrpDPUBLSH = ReentrantCallbackGroup()
        self.DPUBLSH = self.create_publisher(Frame, self.device.devDPUB, 10) # , callback_group=self.cllgrpDPUBLSH

        ## Start threads -----------------------------------------------------------------------------------------
        # start Data Collection Thread
        # self.collect_data = threading.Thread(target=self.collect_DataThread, daemon=True)
        # if not self.softwareTest:
        #      self.collect_data.start()


        # start Data Publish Thread
        self.publish_data = threading.Thread(target=self.publish_DataThread, daemon=True)
        if not self.softwareTest:
            self.publish_data.start()

        # finish node init
        self.get_logger().info("%s Data Node Started" % self.device.devNAME)
        if self.softwareTest:
            self.get_logger().info("Starting Control Node test.\n")
            self.get_logger().info("Start Time:   " + str(time.time()))
            #self.collect_data.start()
            self.publish_data.start()
            time.sleep(1)
            self.allFRMS = True
            #self.collect_data.join()
            self.publish_data.join()
            self.get_logger().info("End Time:   " + str(time.time()))
            self.get_logger().info("Control Node test finished.\n")

    
    # Thread Functions
    # %---------------------------------------------------------------------------------------------------------
    def collect_DataThread(self):
        while not self.allFRMS:
            self.collect_Data()

    def collect_Data(self):
        """Function call to collect data from ethernet port and add to buffer.  
    Receives UDP packet from the data port. Will exit function if time out error is received. 
    This function only supports the RAW data output format and not seperated data.
    The DCA1000 raw data format is: Packet Number [4 Bytes], Byte ID [6 Bytes], Raw Data [max 1456 Bytes].
    Function first extracts packet number to check for dropped packets and will indicate if packets are dropped.
    
    If packets are dropped function will zero fill missing packets. The zero fill feature is not fully functional yet.
    It ensures the data collection will finish but data probably won't be usable. Increase the rcvBUFmult parameter size if
    experiencing packet loss at mmWave object instantiation. 
    Once rawData is extraced from UDP packet. Each byte is added to the class buffer. 
    """
        prevPackNum = self.pckNUM # store previous packet number
        if not self.softwareTest:
            try:
                msg, server = self.dat_socket.recvfrom(1466) # get data from socket
                self.pckNUM = int.from_bytes(msg[:4],"little") # extract current packet number
                rawData = msg[10:] # extrat raw data
            except Exception as e:
                return # exit fucntion call if exception rasied

        else:
            timePerPacket = 25e-5 # approx
            time.sleep(timePerPacket)
            self.pckNUM += 1
            rawData = self.testPacket

        if (self.pckNUM - prevPackNum)>1: # check to see if packet number increased by more than 1 = dropped packet
            self.get_logger().warning(
                "Dropped {packetsDropped} packets. Will fill with zeros".format(packetsDropped=(self.pckNUM - prevPackNum)))
            
            numZeroBytes = (self.pckNUM - prevPackNum)*1456 # calculate the number of missing bytes
            self.frmBUFF.push(b"".join([b"\x00"]*int(numZeroBytes)))
            self.bytWRT += numZeroBytes

        self.frmBUFF.push(rawData)
        self.bytWRT += len(rawData)

    # %---------------------------------------------------------------------------------------------------------
    def publish_DataThread(self):
        while not self.allFRMS:
            self.publish_Data()

    def publish_Data(self):
        """Function call to extract data from class buffer and publish data to ROS topic.  
    Function calls getFrame from class buffer to extract a chunk of data for writing to file. 
    
    In constant frame operation every function call getFrame is called.
    If getFrame does not return None the frame counter is increased by 1. 
    When frame counter is equal to the number of expected frames that means all frames have been 
    collected and function will stop publishing frames and set allFramesCollected to True and device record 
    flag to false.

    In continuous frame operation, as soon as a frame is ready in the buffer it published as long as the 
    record flag is true. 
    """
        self.collect_Data()
        if(self.devDict["frameCfg"]["numFrames"]==0): 
            # continuous modes
            if self.bytWRT >= self.frmSIZE:
                data = bytes(self.frmBUFF.pop(self.frmSIZE)) # get dataChunk from buffer
                frame = [data[i:i + 1] for i in range(0, len(data))]
                self.bytWRT -= self.frmSIZE
                self.frmCNT += 1
            # if not frame==None: # if frame was ready
                if self.dataTest:
                    fileName = self.fileName+str(self.frmCNT)+".bin"
                    filePath = "RadarDataTest_"+str(self.folderCount)+"/"
                    with open(filePath+fileName, "ab") as f: # open file in append binary mode
                        f.write(b"".join(frame))
                    self.get_logger().info("Wrote %d Bytes to file Frame_%d." % (len(frame),self.frmCNT))
                else:
                    msg = Frame()
                    msg.frame_size = self.frmSIZE
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = str(self.frmCNT)
                    msg.frame_data = frame
                    self.DPUBLSH.publish(msg)
                    self.get_logger().info("Published %d Bytes in Frame %d to RADAR topic." % (len(frame),self.frmCNT))
        else:
            if (self.frmCNT >= self.devDict["frameCfg"]["numFrames"]) and not self.allFRMS:
                self.allFRMS = True
                self.device.devFLAGS_["RECRD"] = False
                self.get_logger().info("All Radar Frames Collected, Stopping Data Collection.")
                    
            if not self.allFRMS: # ensures more files aren't written if all frames collected
                frame = self.frmBUFF.getFrame() # get dataChunk from buffer
                if not frame==None: # if frame was ready
                    self.frmCNT += 1
                    msg = Frame()
                    msg.frame_size = self.frmSIZE
                    msg.frame_data = frame
                    self.DPUBLSH.publish(msg)
                    self.get_logger().info("Published Frame %d to topic." % (self.frmCNT))

def main(args=None):
    rclpy.init()
    node = MMWaveDataNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info('Shut down with CTRL-C.\n')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
        exit()
    node.destroy_node()
    rclpy.shutdown()
