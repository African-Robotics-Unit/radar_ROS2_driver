
# processing imports
import numpy as np
import threading
import h5py
import os
import sys

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

filename = "RadarExperiment_"

class MMWaveHDFNode(Node):
    # %---------------------------------------------------------------------------------------------------------
    # Class Parameters
    # %---------------------------------------------------------------------------------------------------------
    
    # M2S2 Device Specification 
    device = None

    # configs
    runDict = None # runtime parameter dictionary 
    devDict = None # device parameter dictionary

    # frame
    frmCNT = 0

    def __init__(self):

        ## Start Node
        self.device = M2S2Device("RADAR")
        super().__init__("%s_hdf5_node" % self.device.devNAME)  

        self.get_logger().info("Starting Radar HDF5 File Recorder.\n") 

        self.runConfigPath = '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgRun.json'
        self.devConfigPath = '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgDev.json'

        # create mmWaveConfig Object and store parameter dicionaries
        self.load_device_params()
        self.load_runtime_params()

        ## Create hdf5_file
        #  Create radar directory
        self.get_logger().info("Creating experiment data directory RadarData.") 
        try:
            os.mkdir("RadarData")
            self.get_logger().info("RadarData Directory Created.") 
        except Exception as e:
            self.get_logger().info("RadarData Directory Already Present")
        
        # create file and basic file structure
        fileCounter = 0
        fileCreated = False
        try:
            while not fileCreated:
                try:
                    self.file = h5py.File("RadarData/" + filename + "_%d.hdf5" % (fileCounter), 'x')
                    fileCreated = True
                except:
                    fileCounter += 1
            self.get_logger().info("Created file: " + filename + "_%d.hdf5\n" % (fileCounter) )

            # create starting heirarchy
            self.get_logger().info("Creating HDF5 Inital Hierarchy." )
            self.dataGrp = self.file.create_group("Data")
            self.get_logger().info("Created Data Group." )

            paramGrp = self.file.create_group("Params")
            self.get_logger().info("Created Params Group.")
            profileCfgGrp = paramGrp.create_group("profileCfg")
            for key in self.devDict["profileCfg"]:
                profileCfgGrp.create_dataset(key,data=self.devDict["profileCfg"][key])

            frameCfgGrp = paramGrp.create_group("frameCfg")
            for key in self.devDict["frameCfg"]:
                frameCfgGrp.create_dataset(key,data=self.devDict["frameCfg"][key])

            channelCfgGrp = paramGrp.create_group("channelCfg")
            for key in self.devDict["channelCfg"]:
                channelCfgGrp.create_dataset(key,data=self.devDict["channelCfg"][key])

            adcCfgGrp = paramGrp.create_group("adcCfg")
            for key in self.devDict["adcCfg"]:
                adcCfgGrp.create_dataset(key,data=self.devDict["adcCfg"][key])

            adcbufCfgGrp = paramGrp.create_group("adcbufCfg")
            for key in self.devDict["adcbufCfg"]:
                adcbufCfgGrp.create_dataset(key,data=self.devDict["adcbufCfg"][key])
    
            commentGrp = self.file.create_group("Comments")
            self.get_logger().info("Created Comments Group.")

            radar=["AWR1843BOOST".encode("ascii")]
            radarTypeDSet = commentGrp.create_dataset("radarType", shape=(len(radar),1), data=radar)    

            setup=["Heart and breathing rate experiments. Trying out different params.".encode("ascii")]       
            setupCommentsDSet = commentGrp.create_dataset("experimentSetup", shape=(len(setup),1), data=setup) 
            self.get_logger().info("Stored Configs and Intial Comments\n") 
       
        except Exception as e:
            self.get_logger().info(str(e))
            self.get_logger().info("Shutting down node")
            exit()

        ## Setup ROS2 Components -------------------------------------------------------------------------------
        self.get_logger().info("Starting Subscriber to radar data topic.") 
        self.cllgrpSUB = MutuallyExclusiveCallbackGroup()
        self.FRMSUB = self.create_subscription(Frame, self.device.devDPUB, self.append_to_file, 10, callback_group=self.cllgrpSUB) # , callback_group=self.cllgrpDPUBLSH
        self.get_logger().info("Radar HDF5 Recorder Node setup finihsed. \n")

    def append_to_file(self, frameMsg):
    
        byteList = frameMsg.frame_data
        timestamp = frameMsg.header.stamp
        frameid = frameMsg.header.frame_id
        
        frameData = np.frombuffer(b"".join(byteList), dtype=np.int16)
        frameGrp = self.dataGrp.create_group("Frame_%s" % (frameid))
        timeGrp = frameGrp.create_group("timeStamps")
        nsecDs = timeGrp.create_dataset("nanosec" , data = timestamp.nanosec, dtype = np.uint32)
        secsDs = timeGrp.create_dataset("seconds" , data = timestamp.sec, dtype = np.int32)
        frameGrp.create_dataset("frameData",data=frameData,dtype=np.int16)

        self.get_logger().info("Stored Frame_%s" % (frameid))
        
    def load_device_params(self):
        # load configs
        self.devDict = mmWaveConfig(self.runConfigPath,self.devConfigPath).mmwaveParamDict

        ## store profile
        # frame dimensions
        self.nChirps = self.devDict["frameCfg"]["numChirps"]
        self.nSamples = self.devDict["profileCfg"]["numAdcSamples"]
        
        # Sample byte size params
        self.bytesPerSample = 2  
        self.realOrComplex = (self.devDict["adcbufCfg"]["adcOutputFmt"]+2) % 3 # 2 bytes if complex, 1 byte if real
        
        # Calculate channels
        recBitMask =  bin(self.devDict["channelCfg"]["rxChannelEn"])[2:] # convert number to bit mask string with bin()
        self.numRx = 0
        for bit in recBitMask:  # count the number of bits = 1 in bit mask to get number of receiver channels enabled
            self.numRx = self.numRx + int(bit)
        self.numTx  = self.devDict["frameCfg"]["chirpEndIndex"] - self.devDict["frameCfg"]["chirpStartIndex"] + 1
        self.nVChannels = self.numTx*self.numRx

    def load_runtime_params(self):
        # load configs
        self.runDict = mmWaveConfig(self.runConfigPath,self.devConfigPath).runtimeParamDict

        # presort_data?


        
  

    def sort_data(self,frame):
        """
             out is returned as a 3D array with the dimensions [numsamples,numChirps,numChannels]
        """
        # calculate frame data size
        totalSamples = self.nSamples*self.nChirps*self.nVChannels*self.realOrComplex
        outputLength = int(totalSamples/2)
        
        # construct complex samples
        out = np.zeros(outputLength,dtype=np.complex64)
        out[ ::2] = frame[ ::4] + 1j*frame[2::4]
        out[1::2] = frame[1::4] + 1j*frame[3::4]

        # reshape the array
        out = np.reshape(out, (self.nSamples, self.nVChannels, self.nChirps),order='F')
        out = np.transpose(out, (0, 2, 1)) 

        return out
    

def main(args=None):
    global filename
    rclpy.init(args=args)
    node = MMWaveHDFNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try: 
        if str(sys.argv[0]) == "":
            filename = "RadarExperiment"
        else:
            filename = filename + str(sys.argv[0])
    except:
        filename = "RadarExperiment"

    try:
        node.get_logger().info("Press Ctrl+C to shutdown.\n")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.\n")
        node.destroy_node()
    rclpy.shutdown()



