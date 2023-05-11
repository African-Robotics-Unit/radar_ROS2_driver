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
from hrm_interfaces.msg import HeartRate

# GUI Imports
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow, QGridLayout, QLabel, QFormLayout
from PyQt5.QtCore import QTimer, QSize, Qt
from PyQt5.QtGui import QFont
import sys

filename = "VitalsExperiment_"
exhaleInhalePeriod = 4 # seconds for inhale and exhale 
heartRate = None
breathingFreq = 1/exhaleInhalePeriod
inhaling = 1

class VitalSignsRecorder(QMainWindow):


    def __init__(self, channel=1):
        super().__init__()

        print("Starting Vital Signs Recorder")

        # setting title
        self.setWindowTitle("Vital Signs Experiment Recorder")
 
        # setting geometry
        self.setGeometry(100, 100, 600, 500)

        # creating a widget object
        self.mainWidget = QWidget()
        self.brtLabelWidget = QLabel("BREATH OUT")
        self.brtLabelWidget.setText("BREATH IN")
        self.brtLabelWidget.setStyleSheet("border: 1px solid black;")
        self.brtLabelWidget.setFont(QFont('Arial', 48))
        self.brtLabelWidget.setAlignment(Qt.AlignCenter)

        # Heart rate
        self.hrtRateLabelWidget = QLabel()
        self.hrtRateLabelWidget.setFont(QFont('Arial', 18))
        self.hrtRateLabelWidget.setText("Hear Rate [bpm]:")
        self.hrtRateLabelWidget.setStyleSheet("border: 0px solid black;")
        self.hrtRateLabelWidget.setAlignment(Qt.AlignCenter)
        
        self.hrtRateWidget = QLabel()
        self.hrtRateWidget.setFont(QFont('Arial', 48))
        self.hrtRateWidget.setStyleSheet("border: 0px solid black;")
        self.hrtRateWidget.setAlignment(Qt.AlignCenter)

        self.hrtWidget = QWidget()
        self.hrtWidget.setStyleSheet("border: 1px solid black;")
        heart_layout = QFormLayout() 
        self.hrtWidget.setLayout(heart_layout)
        heart_layout.addWidget(self.hrtRateLabelWidget)
        heart_layout.addWidget(self.hrtRateWidget)

        # Timer
        self.timerLabelWidget = QLabel()
        self.timerLabelWidget.setFont(QFont('Arial', 18))
        self.timerLabelWidget.setText("Timer [s]:")
        self.timerLabelWidget.setStyleSheet("border: 0px solid black;")
        self.timerLabelWidget.setAlignment(Qt.AlignCenter)

        self.timeWidget = QLabel()
        self.timeWidget.setFont(QFont('Arial', 48))
        self.timeWidget.setText("000")
        self.timeWidget.setStyleSheet("border: 0px solid black;")
        self.timeWidget.setAlignment(Qt.AlignCenter)

        self.timerWidget = QWidget()
        self.timerWidget.setStyleSheet("border: 1px solid black;")
        timer_layout = QFormLayout() 
        self.timerWidget.setLayout(timer_layout)
        timer_layout.addWidget(self.timerLabelWidget)
        timer_layout.addWidget(self.timeWidget)
        
        # Creating a grid layout
        layout = QGridLayout() 
        self.mainWidget.setLayout(layout)
        layout.addWidget(self.brtLabelWidget, 0, 0, 1, 2)
        layout.addWidget(self.hrtWidget, 2, 0, 1, 1)
        layout.addWidget(self.timerWidget, 2, 1, 1, 1)
        self.setCentralWidget(self.mainWidget)

        # showing all the widgets
        self.show()
 
        # setting fixed size of window
        self.setFixedSize(QSize(700, 400))

        self.timer = QTimer()
        self.timer.setInterval(int(exhaleInhalePeriod*1e3/2))
        self.timer.timeout.connect(self.inhaleExhale)
        self.timer.start()

        self.time = 0
        self.timer2 = QTimer()
        self.timer2.setInterval(1000)
        self.timer2.timeout.connect(self.updateTime)
        self.timer2.start()

    def updateTime(self):
        self.time +=1
        timeString = str(self.time)
        timeString = "".join((["0"]*(3-len(timeString)))) + timeString
        self.timeWidget.setText(timeString)   
        self.hrtRateWidget.setText(str(heartRate))     

    def inhaleExhale(self):
        global inhaling
        if inhaling == 1:
            self.brtLabelWidget.setText("BREATH OUT")
            inhaling = 0
        else:
            self.brtLabelWidget.setText("BREATH IN")
            inhaling = 1

class VitalSignsHDFNode(Node):
    # %---------------------------------------------------------------------------------------------------------
    # Class Parameters
    # %---------------------------------------------------------------------------------------------------------
    
    # M2S2 Device Specification 
    radarDev = None
    hrmDev   = None

    # configs
    runDict = None # runtime parameter dictionary 
    devDict = None # device parameter dictionary

    # frame
    frmCNT = 0

    def __init__(self):

        ## Start Node
        self.radDev = M2S2Device("RADAR")
        self.hrmDev = M2S2Device("HRM")

        super().__init__("Vitals_hdf5_node")  

        self.get_logger().info("Starting Radar HDF5 File Recorder.\n") 

        self.runConfigPath = '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgRun.json'
        self.devConfigPath = '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgDev.json'

        # create mmWaveConfig Object and store parameter dicionaries
        self.load_device_params()
        self.load_runtime_params()

        ## Create hdf5_file
        #  Create radar directory
        self.get_logger().info("Creating experiment data directory VitalSignData.") 
        try:
            os.mkdir("VitalSignData")
            self.get_logger().info("VitalSignData Directory Created.") 
        except Exception as e:
            self.get_logger().info("VitalSignData Directory Already Present")
        
        # create file and basic file structure
        fileCounter = 0
        fileCreated = False
        try:
            while not fileCreated:
                try:
                    self.file = h5py.File("VitalSignData/" + filename + "_%d.hdf5" % (fileCounter), 'x')
                    fileCreated = True
                except:
                    fileCounter += 1
            self.get_logger().info("Created file: " + filename + "_%d.hdf5\n" % (fileCounter) )

            ## create starting heirarchy
            # Radar Data
            self.get_logger().info("Creating HDF5 Inital Hierarchy." )
            radarGrp = self.file.create_group("Radar")
            self.get_logger().info("CREATING RADAR GROUP:" )
            self.dataGrp = radarGrp.create_group("Data")
            self.get_logger().info("Created Data Group." )

            paramGrp = radarGrp.create_group("Params")
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

            sortDset = paramGrp.create_dataset("PRESORTED_DATA" , data=self.sort)
            
            commentGrp = radarGrp.create_group("Comments")
            self.get_logger().info("Created Comments Group.")

            radar=["AWR1843BOOST".encode("ascii")]
            radarTypeDSet = commentGrp.create_dataset("radarType", shape=(len(radar),1), data=radar)    

            setup=["Heart and breathing rate experiments. Trying out different params.".encode("ascii")]       
            setupCommentsDSet = commentGrp.create_dataset("experimentSetup", shape=(len(setup),1), data=setup) 
            self.get_logger().info("Stored Configs and Intial Comments\n") 

            # Breathing
            self.get_logger().info("CREATING BREATHING GROUP:" )
            breathGrp = self.file.create_group("Breathing")

            self.get_logger().info("Created Data Group.")
            self.brthDataGrp = breathGrp.create_group("Data") 
            
            self.get_logger().info("Created Params Group.")
            brthParamGrp = breathGrp.create_group("Params")
            brthParamGrp.create_dataset("expected_frequency",data=breathingFreq)

            brthCommentGrp = breathGrp.create_group("Comments") 
            self.get_logger().info("Created Comments Group.") 
            brthComments=["Inhalation and Exhalation timed by GUI commands presented to user.".encode("ascii")]       
            brthCommentsDSet = brthCommentGrp.create_dataset("breathing_comments", shape=(len(brthComments),1), data=brthComments) 
            self.get_logger().info("Stored Intial Comments\n")

            # Heart rate
            self.get_logger().info("CREATING HEART GROUP:" )
            heartGrp = self.file.create_group("Heart")

            self.get_logger().info("Created Data Group.")
            self.heartDataGrp = heartGrp.create_group("Data") 

            hrmCommentGrp = heartGrp.create_group("Comments") 
            self.get_logger().info("Created Comments Group.") 
            hrmComments=["Using Maxim Integrated heart rate monitor product. Set to report heart rate every second. Sitting at rest during experiment.".encode("ascii")]       
            htmCommentsDSet = hrmCommentGrp.create_dataset("heartrate_comments", shape=(len(hrmComments),1), data=hrmComments) 
            self.get_logger().info("Stored Intial Comments\n")
       
        except Exception as e:
            self.get_logger().info(str(e))
            self.get_logger().info("Shutting down node")
            exit()

        ## Setup ROS2 Components -------------------------------------------------------------------------------
        self.get_logger().info("Starting Subscriber to radar data topic.") 
        self.RADcllgrpSUB = MutuallyExclusiveCallbackGroup()
        self.HRMcllgrpSUB = MutuallyExclusiveCallbackGroup()
        self.FRMSUB = self.create_subscription(Frame, self.radDev.devDPUB, self.append_radar_to_file, 10, callback_group=self.RADcllgrpSUB) # , callback_group=self.cllgrpDPUBLSH
        self.get_logger().info("Breathing commands are being recorded.")
        self.get_logger().info("Starting Subscriber to Heart Rate topic.") 
        self.HRMSUB = self.create_subscription(HeartRate, self.hrmDev.devDPUB, self.append_hrm_to_file, 10, callback_group=self.RADcllgrpSUB)
        self.get_logger().info("Vital Signs HDF5 Recorder Node setup finihsed. \n")

    def append_hrm_to_file(self, hrmMsg):
        global heartRate
        heartRate =  hrmMsg.heart_rate
        conf =  hrmMsg.heart_conf
        timestamp = hrmMsg.header.stamp
        sampleid = hrmMsg.header.frame_id

        sampleGrp = self.heartDataGrp.create_group("Sample_%s" % (sampleid))
        timeGrp = sampleGrp.create_group("timeStamps")
        nsecDs = timeGrp.create_dataset("nanosec" , data = timestamp.nanosec, dtype = np.uint32)
        secsDs = timeGrp.create_dataset("seconds" , data = timestamp.sec, dtype = np.int32)
        sampleGrp.create_dataset("heartRateData",data=heartRate,dtype=np.uint32)
        sampleGrp.create_dataset("heartRateConf",data=conf,dtype=np.uint32)

        self.get_logger().info("Stored sample_%s." % (sampleid))


    def append_radar_to_file(self, frameMsg):
    
        byteList = frameMsg.frame_data
        timestamp = frameMsg.header.stamp
        frameid = frameMsg.header.frame_id
        frameData = np.frombuffer(b"".join(byteList), dtype=np.int16)

        frameGrp = self.dataGrp.create_group("Frame_%s" % (frameid))
        if self.sort == True:
            frameGrp.create_dataset("frameData",data=self.sort_data(frameData),dtype=np.complex64)
        else:
            frameGrp.create_dataset("frameData",data=frameData,dtype=np.int16)
        timeGrp = frameGrp.create_group("timeStamps")
        nsecDs = timeGrp.create_dataset("nanosec" , data = timestamp.nanosec, dtype = np.uint32)
        secsDs = timeGrp.create_dataset("seconds" , data = timestamp.sec, dtype = np.int32)
        self.get_logger().info("Stored frame_%s." % (frameid))

        sampleGrp = self.brthDataGrp.create_group("Sample_%s" % (frameid))
        timeGrp = sampleGrp.create_group("timeStamps")
        nsecDs = timeGrp.create_dataset("nanosec" , data = timestamp.nanosec, dtype = np.uint32)
        secsDs = timeGrp.create_dataset("seconds" , data = timestamp.sec, dtype = np.int32)
        sampleGrp.create_dataset("breathRateData",data=inhaling, dtype=np.uint8)
        self.get_logger().info("Stored sample_%s." % (frameid))
        
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
        self.sort = self.runDict["RECORDING"]["SORT"]

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
    rclpy.init()
    node = VitalSignsHDFNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    ros2ExecThread = threading.Thread(target=executor.spin,daemon=True)
    ros2ExecThread.start()

    app = QApplication(sys.argv)
    ard = VitalSignsRecorder()
    ard.show()
    app.exec()

    print("Shutting Down")
    rclpy.shutdown()
    ros2ExecThread.join()




