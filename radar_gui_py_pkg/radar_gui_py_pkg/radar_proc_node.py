# gui imports
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtPrintSupport import *
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import sys

# processing imports
import numpy as np
from numpy import fft
import threading
import cv2 

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

# global variables
frame_data  = None # frame data
frame_id    = None # frame number
runPath     = None # runtime parameter path
devPath     = None # device parameter path
debugString = None # debug string sent to ROS2 out log

class ProcViewer(QMainWindow):

    static_clutter = None
    clutter_collect_flag = False
    clutter_map_collected = False
    clutter_frame_counter = 0

    detection_box_flag = 0

    def __init__(self):
        global runPath
        global devPath

        super().__init__()

        print("Starting Processing Pipeline Viewer")

        ## Setup Radar System Components ----------------------------------------------------------------------
        # store file to config JSONs path in ROS2 parameters
        self.runConfigPath = runPath
        self.devConfigPath = devPath
        self.path = self.runConfigPath
 
        # setting geometry
        self.setGeometry(0,0,1600,900)

        # creating a QPlainTextEdit object
        self.editor = QPlainTextEdit()
        fixedfont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        fixedfont.setPointSize(12)
        self.editor.setFont(fixedfont)
        self.editor.setFixedWidth(500)
        self.editor.setFixedHeight(725)
        if self.path:
            try:
                with open(self.path, 'rU') as f:
                    text = f.read()
            except Exception as e:
                self.dialog_critical(str(e))
            else:
                self.editor.setPlainText(text)  # update the text

        # create colour map
        cm = pg.colormap.get('CET-L9') 

        # creat container widgets
        # main widget
        self.mainwidget = QWidget()
        # plot containers
        self.rdm_container = QWidget()
        self.rdm_container.setStyleSheet("border: 1px solid black;")
        self.cfar_container = QWidget()
        self.cfar_container.setStyleSheet("border: 1px solid black;")
        self.azm_container = QWidget()
        self.azm_container.setStyleSheet("border: 1px solid black;")
        self.ptc_container = QWidget()
        self.ptc_container.setStyleSheet("border: 1px solid black;")
        # control container
        self.controlPanel = QWidget()
        self.controlPanel.setStyleSheet("border: 1px solid black;")

        layout = QGridLayout(self.mainwidget)

        # create rd map widget
        self.rd_imv = pg.ImageView(parent=self.rdm_container, view=pg.PlotItem())
        self.rd_imv.setMaximumHeight(400)
        self.rd_imv.setMinimumHeight(400)
        self.rd_imv.setMaximumWidth(500)
        self.rd_imv.setMinimumWidth(500)
        self.rd_imv.adjustSize()
        self.rd_imv.setColorMap(cm)
        self.rdm_container.setFixedWidth(500) 
        self.rdm_container.setFixedHeight(400)

        # create cfar map widget
        self.cfar_imv = pg.ImageView(parent=self.cfar_container, view=pg.PlotItem())
        self.cfar_imv.setMaximumHeight(400)
        self.cfar_imv.setMinimumHeight(400)
        self.cfar_imv.setMaximumWidth(500)
        self.cfar_imv.setMinimumWidth(500)
        self.cfar_imv.adjustSize()
        self.cfar_imv.setColorMap(cm)
        self.cfar_container.setFixedWidth(500) 
        self.cfar_container.setFixedHeight(400)

        # load and store parameter dicionaries
        self.load_device_params()
        self.load_runtime_params()

        # create azm_map widget
        self.azm_plot = pg.PlotWidget(parent=self.azm_container)
        self.azm_plot.setMaximumHeight(400)
        self.azm_plot.setMinimumHeight(400)
        self.azm_plot.setMaximumWidth(500)
        self.azm_plot.setMinimumWidth(500)
        self.azm_scatter = pg.ScatterPlotItem(
            size=3, brush=pg.mkBrush(255, 255, 255, 255))
        self.azm_plot.addItem(self.azm_scatter)
        self.azm_plot.setXRange(-self.range_max/2, self.range_max/2)
        self.azm_plot.setYRange(0, self.range_max)
        axBottom = self.azm_plot.plotItem.getAxis('bottom') #get x axis
        xTicks = [1, 0.5]
        axBottom.setTickSpacing(xTicks[0], xTicks[1]) #set x ticks (major and minor)
        axLeft = self.azm_plot.plotItem.getAxis('left') #get y axis
        yTicks = [1, 0.5]
        axLeft.setTickSpacing(yTicks[0], yTicks[1]) #set x ticks (major and minor)
        self.azm_plot.showGrid(x=True, y=True, alpha=0.4)
        self.azm_container.setFixedWidth(500) 
        self.azm_container.setFixedHeight(400)

        # create point cloud widget
        self.ptc_plot = gl.GLViewWidget(parent=self.ptc_container)
        self.ptc_plot.setCameraPosition(distance=60, elevation=12) #self.ptc_plot.opts['distance'] = 40
        self.ptc_scatter = gl.GLScatterPlotItem(
            size=3, color=pg.glColor((255, 255, 255)))
        
        # add grids to point cloud
        # gx = gl.GLGridItem()
        # gx.rotate(90, 0, 1, 0)
        # gx.translate(-10, 0, 0)
        # self.ptc_plot.addItem(gx)
        # gy = gl.GLGridItem()
        # gy.rotate(90, 1, 0, 0)
        # gy.translate(0, -10, 0)
        # self.ptc_plot.addItem(gy)
        gz = gl.GLGridItem()
        gz.translate(0, 0, -10)
        self.ptc_plot.addItem(gz)

        self.ptc_plot.setMaximumHeight(400)
        self.ptc_plot.setMinimumHeight(400)
        self.ptc_plot.setMaximumWidth(500)
        self.ptc_plot.setMinimumWidth(500)
    
        self.ptc_plot.addItem(self.ptc_scatter)
        self.ptc_container.setFixedWidth(500) 
        self.ptc_container.setFixedHeight(400)
    

        # create control button
        runConfigButton = QPushButton()
        runConfigButton.setText("Load Runtime Config")
        runConfigButton.clicked.connect(self.load_runtime_params)
        runConfigButton.setFixedWidth(500)
        runConfigButton.setFixedHeight(25)

        # create clutter button
        clutterButton = QPushButton()
        clutterButton.setText("Load Clutter SnapShot")
        clutterButton.clicked.connect(self.load_clutter_map)
        clutterButton.setFixedWidth(500)
        clutterButton.setFixedHeight(25)


        controlLayout = QFormLayout()
        self.controlPanel.setLayout(controlLayout)

        controlLayout.addRow(runConfigButton)
        controlLayout.addRow(clutterButton)
        controlLayout.addRow(self.editor)
 
        # adding to horizont box layout
        layout.addWidget(self.controlPanel,0,0,4,2)
        layout.addWidget(self.rdm_container,0,2,2,2)
        layout.addWidget(self.cfar_container,0,4,2,2)
        layout.addWidget(self.azm_container,2,2,2,2)
        layout.addWidget(self.ptc_container,2,4,2,2)
        self.setCentralWidget(self.mainwidget)
 
        # creating a status bar object
        self.status = QStatusBar()
        self.setStatusBar(self.status)
 
        file_toolbar = QToolBar("File")                     # creating a file tool bar
        self.addToolBar(file_toolbar)                       # adding file tool bar to the window
        file_menu = self.menuBar().addMenu("&File")         # creating a file menu
 
        open_file_action = QAction("Open Config file", self)        # creating a open file action
        open_file_action.setStatusTip("Open Config file")           # setting status tip
        open_file_action.triggered.connect(self.open_config_file)   # adding action to the open file
        file_menu.addAction(open_file_action)                       # adding this to file menu
        file_toolbar.addAction(open_file_action)                    # adding this to tool bar
 
        # similarly creating a save action
        save_file_action = QAction("Save", self)
        save_file_action.setStatusTip("Save current page")
        save_file_action.triggered.connect(self.file_save)
        file_menu.addAction(save_file_action)
        file_toolbar.addAction(save_file_action)
 
        # similarly creating save action
        saveas_file_action = QAction("Save As", self)
        saveas_file_action.setStatusTip("Save current page to specified file")
        saveas_file_action.triggered.connect(self.file_saveas)
        file_menu.addAction(saveas_file_action)
        file_toolbar.addAction(saveas_file_action)
 
        # calling update title method
        self.setWindowTitle("Radar Data Processing Window")
 
        # showing all the components
        self.show()
 
        # setting fixed size of window
        self.setFixedSize(QSize(1600, 900))

        self.timer = QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
        

        print("Viewer Init Finished")

    def dialog_critical(self, s):
        """ 
        Critical dialog method to show errors. Takes in a string to display in the dialog box.  
        """
        dlg = QMessageBox(self)             # creating a QMessageBox object
        dlg.setText(s)                      # setting text to the dlg
        dlg.setIcon(QMessageBox.Critical)   # setting icon to it
        dlg.show()                          # showing it
 
    def open_config_file(self):
        """ 
        Called by file open action. Loads a file to the editor.  
        """
        # getting path and bool value
        path, _ = QFileDialog.getOpenFileName(self, "Open file", "",
                             "Text documents (*.json);All files (*.*)")
 
        if path:
            try:
                with open(path, 'rU') as f:
                    text = f.read()
            except Exception as e:
                self.dialog_critical(str(e))
            else:
                self.path = path                # update path value
                self.editor.setPlainText(text)  # update the text

    def file_save(self):
        """ 
        Called by file save action. Save a file to path.  
        """
 
        # if there is no save path
        if self.path is None:
            return self.file_saveas()   # call save as method
 
        # else call save to path method
        self._save_to_path(self.path)
 
    def file_saveas(self):
        """ 
        Called by file save as action. Save a new file to path.  
        """
 
        # opening path
        path, _ = QFileDialog.getSaveFileName(self, "Save file", "",
                             "Text documents (*.json);All files (*.*)")
 
        # if dialog is cancelled i.e no path is selected
        if not path:
            return
 
        # else call save to path method
        self._save_to_path(path)
 
    def _save_to_path(self, path):
 
        # get the text
        text = self.editor.toPlainText()

        try:
            with open(path, 'w') as f:      # opening file to write
                f.write(text)               # write text in the file

        except Exception as e:
            self.dialog_critical(str(e))    # show error using critical

        else:
            self.path = path # change path

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

        # process data
        self.process_data(frame)
        
        # plot processes
        self.rd_imv.setImage(self.out_rd, axes=None, xvals=None, scale=(self.scaleX,self.scaleY))
        # self.rd_imv.adjustSize()

        self.cfar_imv.setImage(self.out_cfar, axes=None, xvals=None, scale=(self.scaleX,self.scaleY))
        # self.cfar_imv.adjustSize()

        self.azm_scatter.setData(pos=np.array(self.azm_map))
        # color_pts= pg.glColor((255, 255, 255))
        # color_list = [color_pts]*len(self.point_cloud)
        self.point_cloud.append([0,0,0])
        # color_radar =pg.glColor((255, 0, 0))
        # color_list.append(color_radar)
        self.ptc_scatter.setData(pos=np.array(self.point_cloud), color = pg.glColor((255, 255, 255)),size=5,pxMode = True)


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

        # Get chirp parameters 
        chirpTime = (self.devDict['profileCfg']['idleTime'] + self.devDict['profileCfg']['rampEndTime'])*1e-6 # total time between start of one chirp and the next
        sampleRate =  self.devDict['profileCfg']['digOutSampleRate']*1e3    # Sample rate in Sps
        freqSlope =  self.devDict['profileCfg']['freqSlopeConst']*1e12      # Frequency Slope in Hz/s
        bandWidth = (self.nSamples/sampleRate)*freqSlope                    # Bandwidth of chirp that is sampled in Hz
        startFreq = self.devDict['profileCfg']['startFreq']*1e9             # Starting frequency of the chirp in Hz
        centerFreq = (startFreq + bandWidth/2)                              # Midpoint between start and end frequencies. 

        # Get device performance metrics
        self.range_max = 3e8*sampleRate/(2*freqSlope)                   # maximum range (equivalent to range_res*nSamples)
        self.range_res = 3e8/(2*bandWidth)                              # range resolution
        self.doppler_res = 1/(self.nChirps*chirpTime)                   # doppler resolution
        self.velocity_max = ((3e8/(4*centerFreq*chirpTime)))            # Max Velocity
        self.velocity_res =  self.doppler_res*((3e8/(2*centerFreq)))    # doppler resolution

        self.dialog_critical("MAXIMUM RANGE: \t\t{0:.2f}m\nRANGE RESOLUTION: \t\t{1:.2f}m".format(self.range_max,self.range_res))


    def load_runtime_params(self):
        print("Getting runtime config parameters.")
        
        # load configs
        self.file_save()
        self.config = mmWaveConfig(self.runConfigPath,self.devConfigPath)
        self.runDict = self.config.runtimeParamDict

        # store process type
        self.process = self.runDict["PROC_TYPE"]

        # store number of frames to collect for clutter
        self.numClutter = self.runDict["CLUTTER_CONFIG"]["NUM_FRAMES"]

        # store new padding factors for range doppler
        self.rangePadfactor     = self.runDict["ARD_CONFIG"]["RNG_PAD_FACTOR"]
        self.dopplerPadfactor   = self.runDict["ARD_CONFIG"]["DOP_PAD_FACTOR"]
        
        # create windows arrays    
        self.rangeWindowArr     = self.create_window(self.nSamples, self.runDict["ARD_CONFIG"]["RANGE_WINDOW_TYPE"])    
        self.dopplerWindowArr   = self.create_window(self.nChirps , self.runDict["ARD_CONFIG"]["DOPPLER_WINDOW_TYPE"])

        # store CFAR params
        self.cfar_range_gaurd        = self.runDict["CFAR_CONFIG"]["RNG_GUARD_CELLS"]
        self.cfar_range_training     = self.runDict["CFAR_CONFIG"]["RNG_TRAINING_CELLS"]
        self.cfar_doppler_gaurd      = self.runDict["CFAR_CONFIG"]["DOP_GUARD_CELLS"]
        self.cfar_doppler_training   = self.runDict["CFAR_CONFIG"]["DOP_TRAINING_CELLS"]
        self.cfar_pfa                = self.runDict["CFAR_CONFIG"]["PFA"]
        self.cfar_threshold          = self.runDict["CFAR_CONFIG"]["THRESHOLD"]

        # store new plot scale
        self.scaleX         =  self.runDict["PLOT_SCALE_X"]
        self.scaleY         =  self.runDict["PLOT_SCALE_Y"]
        self.orientation    =  self.runDict["PLOT_ORIENTATION"]

        # store new configs for azimuth
        self.azmPhysicalFOV = self.runDict["AZM_CONFIG"]["FOV_PHYSICAL"]
        self.azmProcessFOV  = self.runDict["AZM_CONFIG"]["FOV_PROCESS"]
        self.azmFFTPoints   = self.runDict["AZM_CONFIG"]["FFT_POINTS"]

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

        # Guard statement to make sure process selected is implemented
        if not (self.process in ["RANGE_DOPPLER","CFAR","RANGE_AZIMUTH","POINT_CLOUD"]):
            print("ERROR: Process selected not recognised.")
            out = np.zeros((self.nSamples, self.nChirps))
            return out

        if self.clutter_collect_flag:
            self.create_clutter_map(data)

        ## create arrays
        r_fft  = np.zeros((self.nSamples*self.rangePadfactor, self.nChirps, self.nVChannels)
                    , dtype=np.complex64) # Range FFT maps, 1 Map per virtual channel. Input to doppler FFT.
        rd_fft  = np.zeros((self.nSamples*self.rangePadfactor, self.nChirps*self.dopplerPadfactor, self.nVChannels)
                    , dtype=np.complex64) # Range-Doppler FFT maps, 1 Map per virtual channel. 
        rd_map = np.zeros((self.nSamples*self.rangePadfactor, self.nChirps*self.dopplerPadfactor)
                    , dtype=np.complex64) # Range-Doppler FFT map summed accross all virtual channels. Used as input to CFAR
        cfar_map = np.zeros((self.nSamples*self.rangePadfactor, self.nChirps*self.dopplerPadfactor)
                    ,dtype=np.uint8) # CFAR Map used to determine which samples to use for Azimuth and elevation processing 
         
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
            rd_fft[:,:,channel] = fft.fftshift(fft.fft(r_fft[:,:,channel],self.nChirps*self.dopplerPadfactor,1),1)
            
            rd_map = rd_map + np.abs(rd_fft[:,:,channel])

        rd_map = 20*np.log10(np.abs(rd_map)/np.max(np.max(np.abs(rd_map))))

        cfar_map = self.cfar_detector(rd_map)

        # find clusters in cfar map
        img = cv2.convertScaleAbs(cfar_map)
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        global debugString
        debugString = "Number of cfar clusters is: %d" % (len(contours))

        # find centroids of clusters
        centroid_list = []
        refined_cfar = np.zeros((self.nSamples*self.rangePadfactor, self.nChirps*self.dopplerPadfactor)
                    ,dtype=np.uint8)
        for c in contours:
            # calculate moments for each contour
            M = cv2.moments(c)
            
            # calculate x,y coordinate of center
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centroid = [cX,cY]
                centroid_list.append(centroid)
                rad = np.sqrt(M["m00"]/np.pi)
                cv2.circle(refined_cfar, (cX, cY), int(rad*0.75), 1, -1)         

        # calculate range azimuth values
        detections = np.argwhere(refined_cfar)
        self.azm_map = []
        self.point_cloud = []
        for detection in detections:
            sample = detection[0]
            chirp = detection[1]
            
            # get phase difference between the antennae for tx1
            short_base1      = np.angle(rd_fft[sample,chirp,0]) - np.angle(rd_fft[sample,chirp,1])
            short_base1_AOA  = np.arcsin(short_base1/np.pi)
            short_base2      = np.angle(rd_fft[sample,chirp,1]) - np.angle(rd_fft[sample,chirp,2])
            short_base2_AOA  = np.arcsin(short_base2/np.pi)
            short_base3      = np.angle(rd_fft[sample,chirp,2]) - np.angle(rd_fft[sample,chirp,3])
            short_base3_AOA  = np.arcsin(short_base3/np.pi)
            azm_angle_rad1 = np.average([short_base1_AOA,short_base2_AOA,short_base3_AOA])

            # # get phase difference between the antennae for tx2
            # short_base1      = np.angle(rd_fft[sample,chirp,4]) - np.angle(rd_fft[sample,chirp,5])
            # short_base1_AOA  = np.arcsin(short_base1/np.pi)
            # short_base2      = np.angle(rd_fft[sample,chirp,5]) - np.angle(rd_fft[sample,chirp,6])
            # short_base2_AOA  = np.arcsin(short_base2/np.pi)
            # short_base3      = np.angle(rd_fft[sample,chirp,6]) - np.angle(rd_fft[sample,chirp,7])
            # short_base3_AOA  = np.arcsin(short_base3/np.pi)
            # azm_angle_rad2 = np.average([short_base1_AOA,short_base2_AOA,short_base3_AOA])

            # # get phase difference between the antennae for tx3
            # short_base1      = np.angle(rd_fft[sample,chirp,8]) - np.angle(rd_fft[sample,chirp,9])
            # short_base1_AOA  = np.arcsin(short_base1/np.pi)
            # short_base2      = np.angle(rd_fft[sample,chirp,9]) - np.angle(rd_fft[sample,chirp,10])
            # short_base2_AOA  = np.arcsin(short_base2/np.pi)
            # short_base3      = np.angle(rd_fft[sample,chirp,10]) - np.angle(rd_fft[sample,chirp,11])
            # short_base3_AOA  = np.arcsin(short_base3/np.pi)
            # azm_angle_rad3 = np.average([short_base1_AOA,short_base2_AOA,short_base3_AOA])

            # average results from 3 virtual receiver patters
            # azm_angle_rad = np.average([azm_angle_rad1,azm_angle_rad2,azm_angle_rad3])
            azm_angle_rad = azm_angle_rad1
            
            # # get phase difference between the end two antennae  
            # long_base       = np.angle(rd_fft[sample,chirp,0]) - np.angle(rd_fft[sample,chirp,11]) 
            # long_base_AOA_array = np.arcsin((long_base+2*np.pi*np.arange(-4,5))/(7*np.pi))
            # long_base_AOA_array = long_base_AOA_array[~np.isnan(long_base_AOA_array)]

            # # compare long base to short base
            # smallest_error_index  = np.argmin(np.abs(long_base_AOA_array - short_base_AOA))
            # azm_angle_rad = long_base_AOA_array[smallest_error_index]  

            # account for zero padding and inverted range vs sample index relationship (i.e. sample at 256 = 0m range)
            num_range_bins = self.nSamples*self.rangePadfactor                  # work out number of bins (affected by zero padding)
            range_bins = np.flip(np.linspace(0,self.range_max,num_range_bins))  # create array of range inverted to match index   

            # calculate x-y position
            x, y = self.azimuth_to_xy(azm_angle_rad,range_bins[sample])
            self.azm_map.append([x,y])


            # get short bases for elevation
            short_base1      = np.angle(rd_fft[sample,chirp,4]) - np.angle(rd_fft[sample,chirp,2])
            short_base1_EAOA = np.arcsin(short_base1/np.pi)
            short_base2      = np.angle(rd_fft[sample,chirp,5]) - np.angle(rd_fft[sample,chirp,3])
            short_base2_EAOA = np.arcsin(short_base2/np.pi)
            short_base3      = np.angle(rd_fft[sample,chirp,6]) - np.angle(rd_fft[sample,chirp,8])
            short_base3_EAOA = np.arcsin(short_base3/np.pi)
            short_base4      = np.angle(rd_fft[sample,chirp,7]) - np.angle(rd_fft[sample,chirp,9])
            short_base4_EAOA = np.arcsin(short_base4/np.pi)
            elv_angle_rad    = np.average([short_base1_EAOA,short_base2_EAOA,short_base3_EAOA,short_base4_EAOA,])  

            # calculate x-y-z position
            x, y, z = self.spherical_to_cartesian(azm_angle_rad,elv_angle_rad,range_bins[sample])
            xyz_co_ord = [x,y,z]
            self.point_cloud.append(xyz_co_ord)

        # select map to set to output
        rd_map = np.transpose(rd_map)
        rd_map = np.fliplr(rd_map)  
        
        # cfar_map = np.transpose(cfar_map)
        # cfar_map = np.fliplr(cfar_map)  
        refined_cfar = np.transpose(refined_cfar)
        refined_cfar = np.fliplr(refined_cfar)  
        

        # format the output map orientation 
        if self.runDict["PLOT_ORIENTATION"] == "HORIZONTAL":
            rd_map = np.transpose(rd_map)
            # cfar_map = np.transpose(cfar_map)
            refined_cfar = np.transpose(refined_cfar)

        self.out_cfar = refined_cfar
        self.out_rd = rd_map


    def cfar_detector(self, rdm_frame):

        # create a cfar training kernel
        p_kernel = np.ones(
            (1 + 2*(self.cfar_range_gaurd + self.cfar_range_training),
             1 + 2*(self.cfar_doppler_gaurd + self.cfar_doppler_training)))
        
        # set the middle of the kernel to zero to create guard cells
        guard = np.zeros((2*self.cfar_range_gaurd+1,2*self.cfar_doppler_gaurd+1))
        p_kernel[self.cfar_range_training  :self.cfar_range_training+1+2*self.cfar_range_gaurd,
                 self.cfar_doppler_training:self.cfar_doppler_training+1+2*self.cfar_doppler_gaurd] = guard        
        
        # additional cfar params 
        pfa = self.cfar_pfa                                             # probability of false alarms
        num_train_cells = np.sum(np.sum(p_kernel))                      # number of training cells
        alpha = num_train_cells * (pfa ** (-1 / num_train_cells) - 1)   # threshold gain
        dims = np.shape(rdm_frame)
        kernel = np.zeros(dims)

        # squaring the rdm_frame
        rdm_power = np.square(np.abs(rdm_frame))
        
        # zero pad kernel for fft
        kernel[0 : np.size(p_kernel, 0), 0 : np.size(p_kernel, 1)] = p_kernel
        kernel = kernel / num_train_cells
    
        mask = fft.fft2(kernel)                                            # put mask in frequency
        noise = fft.ifft2(np.multiply(np.conj(mask), fft.fft2(rdm_power))) # convolution done in frequency
        row_shift = int( np.floor(np.size(p_kernel, 0) // 2))              # shift that has to be accounted for after the convolution
        noise = np.roll(noise, row_shift, 0)                               # account for shift introduced by convolution

        # threshold exceedance
        indices = rdm_power > (noise * alpha + self.cfar_threshold) # does the RD map exceed the cfar threshold at any points?
        detection_indices = np.argwhere(indices)                    # list of [row_idx, col_idx] where there is a possible detections
        n = np.size(detection_indices, 0)                           # number of possible detections
        
        local_max_indices = np.zeros(dims)
        for i in range(n):
            row_col_idx = detection_indices[i, :]
            row = row_col_idx[0]
            col = row_col_idx[1]
            local_max_indices[row, col] = 1

        detections = local_max_indices  # matrix with ones where detection occured
        return detections

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

    def create_clutter_map(self, data):
        print("Calibrating for clutter: {0:.2f}%%".format(self.clutter_frame_counter*100/self.numClutter))
        if self.clutter_frame_counter == 0:
            self.static_clutter = data
        else:
            self.static_clutter = self.static_clutter + data
        self.clutter_frame_counter+=1
        if self.clutter_frame_counter > self.numClutter:
            self.clutter_collect_flag = False
            self.clutter_map_collected = True
            self.static_clutter=self.static_clutter/self.numClutter
            print("FINISHED CALIBRATION\n")

    def azimuth_to_xy(self,angle_rad,range_m):
        x = range_m*np.sin(angle_rad)
        y = range_m*np.cos(angle_rad)
        return x,y

    def spherical_to_cartesian(self, azm_rad, elv_rad, range_m):
        x = range_m*np.cos(elv_rad)*np.sin(azm_rad)
        y = range_m*np.cos(elv_rad)*np.cos(azm_rad)
        z = range_m*np.sin(elv_rad)
        return x,y,z

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
        global runPath
        global devPath

        ## Start Node
        self.device = M2S2Device("RADAR")
        super().__init__("%s_ard_node" % self.device.devNAME)         

        self.declare_parameter('run_cfg_file_path', '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgRun.json')
        self.declare_parameter('dev_cfg_file_path', '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgDev.json')
        self.runConfigPath = self.get_parameter('run_cfg_file_path').get_parameter_value().string_value
        self.devConfigPath = self.get_parameter('dev_cfg_file_path').get_parameter_value().string_value

        runPath = self.runConfigPath
        devPath = self.devConfigPath

        ## Setup ROS2 Components -------------------------------------------------------------------------------
        self.cllgrpSUB = MutuallyExclusiveCallbackGroup()
        self.FRMSUB = self.create_subscription(Frame, self.device.devDPUB, self.process_callback, 10, callback_group=self.cllgrpSUB) # , callback_group=self.cllgrpDPUBLSH
        self.get_logger().info("RADAR_ard_node setup finihsed")

    def process_callback(self, frame):
        global frame_data
        global frame_id
        global debugString
        frame_data = frame.frame_data
        frame_id = frame.header.frame_id
        if debugString != None:
            self.get_logger().info(debugString)
            debugString = None
        
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



