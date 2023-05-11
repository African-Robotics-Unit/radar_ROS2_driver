# gui imports
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow, QGridLayout, QPushButton, QFormLayout
from PyQt5.QtCore import QTimer, QSize, Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg
import sys

# processing imports
import numpy as np
from numpy import fft
import threading

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

# frame data
frame_data = None
frame_id = None

class ProcViewer(QMainWindow):

    static_clutter = None
    clutter_collect_flag = False
    clutter_map_collected = False
    clutter_frame_counter = 0

    def __init__(self):
        super().__init__()

        print("Starting Processing Pipeline Viewer")

        ## Setup Radar System Components ----------------------------------------------------------------------
        # store file to config JSONs path in ROS2 parameters
        self.runConfigPath = '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgRun.json'
        self.devConfigPath = '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgDev.json'

        # load and store parameter dicionaries
        self.load_device_params()
        self.load_runtime_params()
        
        # setting title
        self.setWindowTitle("%s Map" % self.process)
 
        # setting geometry
        self.setGeometry(100, 100, 600, 500)

        # creat main widget
        self.mainwidget = QWidget()
        self.controlPanel = QWidget()

        # create map widget
        self.imv = pg.ImageView(view=pg.PlotItem())
 
        # setting color map to the image view
        cm = pg.colormap.get('CET-L9') # prepare a linear color map
        # cm.reverse()   
        self.imv.setColorMap(cm)

        # create control button
        runConfigButton = QPushButton()
        runConfigButton.setText("Load Runtime Config")
        runConfigButton.clicked.connect(self.load_runtime_params)

        # create clutter button
        clutterButton = QPushButton()
        clutterButton.setText("Load Clutter SnapShot")
        clutterButton.clicked.connect(self.load_clutter_map)

        controlLayout = QFormLayout()
        self.controlPanel.setLayout(controlLayout)

        controlLayout.addRow(runConfigButton)
        controlLayout.addRow(clutterButton)
 
        # Creating a grid layout
        layout = QGridLayout() 
        self.mainwidget.setLayout(layout)
        layout.addWidget(self.imv, 0, 1, 3, 3)
        layout.addWidget(self.controlPanel, 0, 4, 1, 1)
        self.setCentralWidget(self.mainwidget)
 
        # showing all the widgets
        self.show()
 
        # setting fixed size of window
        self.setFixedSize(QSize(1200, 1000))

        print("Starting Viewer Timer")
        self.timer = QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

        print("Viewer Init Finished")

    def load_clutter_map(self):
        print("Loading clutter")
        self.clutter_collect_flag = True
        self.static_clutter = None
        self.clutter_map_collected = False
        self.clutter_frame_counter = 0

    def update_plot_data(self):
        
        # load data from global variable
        if frame_data == None:
            return

        byteList = frame_data
        data = np.frombuffer(b"".join(byteList), dtype=np.int16)
        
        # sort the data
        frame = self.sort_data(data)
        
        # process the data
        plot = self.process_data(frame)

        # plot the data
        self.img = plot
        self.imv.setImage(self.img, axes=None, xvals=None, scale=(self.scaleX,self.scaleY))
        # print("Viewer Displaying Frame %s" % (frame_id))

    def load_device_params(self):
        # load configs
        print("Getting device config parameters.")
        self.config = mmWaveConfig(self.runConfigPath,self.devConfigPath)
        self.devDict =  self.config.mmwaveParamDict

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

        print("Getting runtime config parameters.")
        # load configs
        self.config = mmWaveConfig(self.runConfigPath,self.devConfigPath)
        self.runDict = self.config.runtimeParamDict

        # store process type
        self.process = self.runDict["PROC_TYPE"]

        # store new padding factors for range doppler
        self.rangePadfactor     = self.runDict["ARD_CONFIG"]["RNG_PAD_FACTOR"]
        self.dopplerPadfactor   = self.runDict["ARD_CONFIG"]["DOP_PAD_FACTOR"]
        
        # create windows  arrays    
        self.rangeWindowArr     = self.create_window(self.nSamples, self.runDict["ARD_CONFIG"]["RANGE_WINDOW_TYPE"])    
        self.dopplerWindowArr   = self.create_window(self.nChirps , self.runDict["ARD_CONFIG"]["DOPPLER_WINDOW_TYPE"])

        # store new plot scale
        self.scaleX         =  self.runDict["PLOT_SCALE_X"]
        self.scaleY         =  self.runDict["PLOT_SCALE_Y"]
        self.orientation    =  self.runDict["PLOT_ORIENTATION"]

        # store new configs for azimuth
        # self.azmWindow = self.runDict["AZM_CONFIG"]["WINDOW_TYPE"]
        self.azmFFTPoints = self.runDict["AZM_CONFIG"]["FFT_POINTS"]

        # store new configs for elevation
        # self.elvWindow = self.runDict["ELV_CONFIG"]["WINDOW_TYPE"]
        self.elvFFTPoints = self.runDict["ELV_CONFIG"]["FFT_POINTS"]

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

    def process_data(self,data):

        # create arrays
        r_fft  = np.zeros((self.nSamples*self.rangePadfactor, self.nChirps, self.nVChannels), dtype=np.complex64)
        rd_fft  = np.zeros((self.nSamples*self.rangePadfactor, self.nChirps*self.dopplerPadfactor, self.nVChannels), dtype=np.complex64)

        if self.process == "RANGE_DOPPLER":
            out = np.zeros((self.nSamples*self.rangePadfactor, self.nChirps*self.dopplerPadfactor), dtype=np.complex64)
        elif self.process == "RANGE_AZIMUTH":
            out = np.zeros((self.nSamples*self.rangePadfactor, self.azmFFTPoints), dtype=np.complex64)
        elif self.process == "RANGE_ELEVATION":
            out = np.zeros((self.nSamples*self.rangePadfactor, self.elvFFTPoints), dtype=np.complex64)
        else:
            print("ERROR: Process selected not recognised.")
            out = np.zeros((self.nSamples, self.nChirps), dtype=np.complex64)
            return out
            
        # do range doppler for each channel
        for channel in range(self.nVChannels):
            # remove clutter
            if self.clutter_map_collected:
                doppler_bins = range(self.nChirps)#32 #range((int(self.nChirps/2)-1),(int(self.nChirps/2)+2),1)
                data[:,doppler_bins,channel] = data[:,doppler_bins,channel] - self.static_clutter[:,doppler_bins,channel]

            # window each chirp and do range fft
            for chirp in range(self.nChirps):
                r_fft[0:self.nSamples,chirp,channel] = data[:,chirp,channel]*self.rangeWindowArr
            r_fft[:,:,channel] = (fft.fft(r_fft[:,:,channel],self.nSamples*self.rangePadfactor,0))
            

            # window accross chirp and do range fft
            for sample in range(self.nSamples*self.rangePadfactor): # r_fft[:,0,0])
                r_fft[sample,0:self.nChirps,channel] = r_fft[sample,:,channel]*self.dopplerWindowArr
            rd_fft[:,:,channel] = (fft.fft(r_fft[:,:,channel],self.nChirps*self.dopplerPadfactor,1))
            
            # sum accross channels for better SNR
            if self.process == "RANGE_DOPPLER":
                out = out + fft.fftshift(np.abs(rd_fft[:,:,channel]),1)

        if self.process == "RANGE_DOPPLER":
            out = np.transpose(out)
            out = np.fliplr(out)
            out = np.transpose(out)

            out = np.transpose(out)

        elif self.process == "RANGE_AZIMUTH":
            # calculate range azimuth values
            for chirp in range(self.nChirps*self.dopplerPadfactor):
                tx02= rd_fft[:,chirp,[0,1,2,3,8,9,10,11]]
                out = out + np.abs(fft.fftshift(fft.fft(tx02,self.azmFFTPoints,1),1))    

        elif self.process == "RANGE_ELEVATION":
            # calculate range elevation values
            for chirp in range(self.nChirps*self.dopplerPadfactor):
                vrp42 = np.abs((fft.fft(rd_fft[:,chirp, [4,2]],self.elvFFTPoints,1))) 
                vrp53 = np.abs((fft.fft(rd_fft[:,chirp, [5,3]],self.elvFFTPoints,1)))
                vrp68 = np.abs((fft.fft(rd_fft[:,chirp, [6,8]],self.elvFFTPoints,1)))
                vrp79 = np.abs((fft.fft(rd_fft[:,chirp, [7,9]],self.elvFFTPoints,1)))
                out = out + np.abs(fft.fftshift((vrp42 + vrp53 + vrp68 + vrp79),1))   
        
        if self.clutter_collect_flag:
            print("Calibrating for clutter: {0:.2f}%%".format(self.clutter_frame_counter*100/50))
            if self.clutter_frame_counter == 0:
                self.static_clutter = data
            else:
                self.static_clutter = self.static_clutter + data
            self.clutter_frame_counter+=1
            if self.clutter_frame_counter > 50:
                self.clutter_collect_flag = False
                self.clutter_map_collected = True
                self.static_clutter=self.static_clutter/50
                print("FINISHED CALIBRATION\n")

        out = 20*np.log10(np.abs(out)/np.max(np.max(np.abs(out))))

        if self.runDict["PLOT_ORIENTATION"] == "HORIZONTAL":
            out = np.transpose(out)
        return out
    
    def create_window(self, windowSize, windoWType):
        # create window      
        if windoWType   == "BLACKMAN":
            windowArray = np.blackman(windowSize)

        elif windoWType ==  "HANNING":
            windowArray = np.hanning(windowSize)
            
        elif windoWType == "BARTLETT":
            windowArray = np.bartlett(windowSize)
        
        elif windoWType == "HAMMING":
            windowArray = np.hamming(windowSize)
        else:
            # rectangular window
            windowArray = np.ones(windowSize)
        
        return windowArray


class MMWaveProcNode(Node):
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
        super().__init__("%s_ard_node" % self.device.devNAME)         

        ## Setup ROS2 Components -------------------------------------------------------------------------------
        self.cllgrpSUB = MutuallyExclusiveCallbackGroup()
        self.FRMSUB = self.create_subscription(Frame, self.device.devDPUB, self.process_callback, 10, callback_group=self.cllgrpSUB) # , callback_group=self.cllgrpDPUBLSH
        self.get_logger().info("RADAR_ard_node setup finihsed")

    def process_callback(self, frame):
        global frame_data
        global frame_id
        frame_data = frame.frame_data
        frame_id = frame.header.frame_id
        # self.get_logger().info("Frame %s Data Loaded." % (frame_id))
    

def main(args=None):
    rclpy.init()
    node = MMWaveProcNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    ros2ExecThread = threading.Thread(target=executor.spin,daemon=True)
    ros2ExecThread.start()

    app = QApplication(sys.argv)
    ard = ProcViewer()
    ard.show()
    app.exec()

    print("Shutting Down")
    rclpy.shutdown()
    ros2ExecThread.join()



