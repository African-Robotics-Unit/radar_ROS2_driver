# Standard Python Libray
import json # library for writing and parsing dicts to json files 

class mmWaveConfig():
    # %---------------------------------------------------------------------------------------------------------
    # Class Parameters
    # %---------------------------------------------------------------------------------------------------------
    # Runtime dict stores parameters for how data and commands are sent to and received from device at runtime
    # equivalent to mmwaveConfig.txt file in TI mmwave platforms tool box
    runtimeParamDict = None

    # mmWave dict stores device parameters for configuration of waveform, adc and data port outputs etc. 
    # equivalent to profile_monitor_xwrXXXX.cfg files in TI mmwave platforms tool box
    mmwaveParamDict = None

    # list of available config commands for mmwaveDevice used to check contents of json are correct
    # key is command, value is number of settings which is equal to len(settingsDict)
    commandList = {'dfeDataOutputMode'  : 1, 
                    'channelCfg'        : 3, 
                    'adcCfg'            : 2, 
                    'adcbufCfg'         : 5, 
                    'profileCfg'        : 14, 
                    'chirpCfg_0'        : 8, 
                    'chirpCfg_1'        : 8, 
                    'chirpCfg_2'        : 8, 
                    'frameCfg'          : 7, 
                    'lowPower'          : 2, 
                    'lvdsStreamCfg'     : 4, 
                    'calibMonCfg'       : 2}

    # list of available runtime config options used to check runtime file format correctness
    runtimeList = ['VARIANT', 
                    'COMMAND_PORT', 
                    'FRAME_BUFFER_MLT',
                    'UDP_BUFFER_MLT',
                    'DCA_DATA_FORMAT_MODE', 
                    'DCA_LVDS_LANE_MODE',
                    'PROC_TYPE',
                    'PLOT_ORIENTATION',
                    'PLOT_SCALE_X',
                    'PLOT_SCALE_Y',
                    'CLUTTER_CONFIG',
                    'ARD_CONFIG',
                    'CFAR_CONFIG',
                    'AZM_CONFIG',
                    'ELV_CONFIG']

    # %---------------------------------------------------------------------------------------------------------
    # Class Functions
    # %---------------------------------------------------------------------------------------------------------
    # Class Constructor
    def __init__(self, runtimeConfigPath = None, mmwaveConfigPath = None):
        """Initializes the config object which stores all required configuration parameters.
        
        Function either initializes configs to default values or loads files from json. All 
        configs are stored as dictionaries for easy access to values. 
        Dictionaries are accessed by called mmWaveConfigObj.runtimeParamDict for runtime parameters and
        mmWaveConfigObj.mmwaveParamDict for mmwave sensor parameters
            
        :param string runtimeConfigPath: the path to mmWaveRuntimeConfig.json  (Default = None, default configs will be loaded)
        :param string mmwaveConfigPath: the path to mmWaveDeviceConfig_XWRXXXX.json   (Default = None, default configs will be loaded)
        """
        if runtimeConfigPath == None:
            # default values stored in class (can use as a template for making your own JSONs)
            self.runtimeParamDict =   {
                        'VARIANT'               : "IWR6843",    # radar board type
                        'COMMAND_PORT'          : "COM4",       # UART command port designation
                        'FRAME_BUFFER_MLT'      : 3,            # Size of circular frame buffer class in no. of frames                             
                        'UDP_BUFFER_MLT'        : 3,            # Multiplies default UDP packet buffer size
                        'DCA_DATA_FORMAT_MODE'  : 3,            # 1: 12bit, 2: 14bit, 3:16bit
                        'DCA_LVDS_LANE_MODE'    : 2,            # 2: 2 Lane LVDS, 4: 4 Lane LVDS

                        "PROC_TYPE"         : "RANGE_DOPPLER",  # radar processing type [RANGE_DOPPLER, RANGE_AZIMUTH, RANGE_ELEVATION]  
                        "PLOT_ORIENTATION"  : "HORIZONTAL",     # transpose 2d plot. Just rotate 90deg ["HORIZONTAL, VERTICAL"]
                        "PLOT_SCALE_X"      : 1,                # Plot scale in the x dimension
                        "PLOT_SCALE_Y"      : 1,                # Plot scale in the Y dimension

                        "CLUTTER_CONFIG" : {
                            "NUM_FRAMES"     : 50
                        },

                        "ARD_CONFIG" : {
                            "RANGE_WINDOW_TYPE"     : "BLACKMAN", # Choose window type for range FFT
                            "DOPPLER_WINDOW_TYPE"   : "BLACKMAN", # Choose window type for doppler FFT
                            "RNG_PAD_FACTOR"        : 1,          # Multiplies the number of range FFT points to zero pad FFT 
                            "DOP_PAD_FACTOR"        : 1           # Multiplies the number of doppler FFT points to zero pad FFT 
                        },

                        "CFAR_CONFIG" : {
                            "RNG_GUARD_CELLS"       : 1,
                            "DOP_GUARD_CELLS"       : 2,
                            "RNG_TRAINING_CELLS"    : 1,
                            "DOP_TRAINING_CELLS"    : 2,
                            "PFA"                   : 10e-2,
                            "THRESHOLD"             : -8   
                        },

                        "AZM_CONFIG" : {
                            "FOV_PHYSICAL"  : 180,
                            "FOV_PROCESS"   : 100,
                            "FFT_POINTS"    : 256
                        },

                        "ELV_CONFIG" : {
                            "WINDOW_TYPE"   : "BLACKMAN",
                            "FFT_POINTS"    : 128           # Number of FFT points. Number of FFT points will be 12 + zero padding = FFT_POINTS
                        }
                        }
        else:
            # if path given, instead set runtimeDict to parsed json file
            self.parse_runtimeConfigJSON(runtimeConfigPath)

        if mmwaveConfigPath == None:
            # default values stored in class (can use as a template for making your own JSONs)
            self.mmwaveParamDict =   {
                        'dfeDataOutputMode':
                            {'frmMode'          : 1},       # modeType 1 = frameBased Chirps, 2 = continuous chirping, 3 = advanced frame config

                        'channelCfg':
                            {'rxChannelEn'      : 15,       # bit mask 15 = 1111 i.e. takes values 1-15
                             'txChannelEn'      : 7,        # bit mask 7 =  111  i.e. takes values 1-7
                             'casMode'          : 0},       # 0-en, 1-dis

                        'adcCfg':
                            {'numADCBits'       : 2,        # 0=12, 1=14, 2=16
                             'adcOutputFmt'     : 1},       # 0=real, 1=complex1, 2=complex2
                        
                        'adcbufCfg':
                            {'subFrameIdx'      : -1,       # -1 for legacy mode
                             'adcOutputFmt'     : 0,        # 0=complex, 1=real
                             'SampleSwap'       : 1,        # 0 = I in LSB, Q in MSB ; 1 = Q in LSB, I in MSB
                             'chanInterleave'   : 1,        # 0 = Interleaved (XWR14xx only), 1 = not-interleaved
                             'chirpThreshold'   : 1},       # 0-8 = DSP for 1D FFT, 1 = HWA for 1D FFT

                        'profileCfg':
                            {'profileId'        : 0, 
                             'startFreq'        : 60,       # in GHz
                             'idleTime'         : 117,      # in us
                             'adcStartTime'     : 7,        # in us
                             'rampEndTime'      : 13.12,    # in us
                             'txOutPower'       : 0,        # in dB
                             'txPhaseShifter'   : 0, 
                             'freqSlopeConst'   : 38.11,    # in MHz/us 
                             'txStartTime'      : 1,        # in us
                             'numAdcSamples'    : 64,       
                             'digOutSampleRate' : 12500,    # ksps 
                             'hpfCornerFreq1'   : 0,        # hpf1 0=175KHz, 1=235KHz, 2=350KHz, 3=700KHz
                             'hpfCornerFreq2'   : 0,        # hpf2 0=350KHz, 1=700KHz, 2=1.4MHz, 3=2.8MHz
                             'rxGain'           : 158},     # in dB

                        'chirpCfg_0':
                            {'chirpStartIndex'  : 0,        # 0-511
                             'chirpEndIndex'    : 0,        # 0-511
                             'profileId'        : 0,        # match profileCfg -> profileID
                             'startFreqVar'     : 0,        
                             'freqSlopeVar'     : 0,
                             'idleTimeVar'      : 0, 
                             'ADCStartTimeVar'  : 0, 
                             'chirpAntennaEnMask': 1},

                        'chirpCfg_1':
                            {'chirpStartIndex'  : 1,        # 0-511
                             'chirpEndIndex'    : 1,        # 0-511
                             'profileId'        : 0,        # match profileCfg -> profileID
                             'startFreqVar'     : 0,        
                             'freqSlopeVar'     : 0,
                             'idleTimeVar'      : 0, 
                             'ADCStartTimeVar'  : 0, 
                             'chirpAntennaEnMask': 4},

                        'chirpCfg_2':
                            {'chirpStartIndex'  : 1,        # 0-511
                             'chirpEndIndex'    : 1,        # 0-511
                             'profileId'        : 0,        # match profileCfg -> profileID
                             'startFreqVar'     : 0,        
                             'freqSlopeVar'     : 0,
                             'idleTimeVar'      : 0, 
                             'ADCStartTimeVar'  : 0, 
                             'chirpAntennaEnMask': 4},

                        'frameCfg':
                            {'chirpStartIndex'  : 0,        # 0-511
                             'chirpEndIndex'    : 1,        # 0-511
                             'numChirps'        : 32,       # 1-255 chirps
                             'numFrames'        : 100,      # 0-65535
                             'framePeriod'      : 100,      # in ms
                             'triggerSelect'    : 1,        # 1=Software, 2=hardware
                             'frameTriggerDelay': 0},       # in 
                             
                        'lowPower':
                            {'dontCare'         : 0,        # always 0
                             'adcMode'          : 0},       # 0=regular, 1=lowPower
                        
                        'lvdsStreamCfg':
                            {'subFrameIdx '     : -1,       # -1 for legacy
                             'enableHeader'     : 0,        # 1=HSI Header enabled, 0 HSI Header disabled
                              'dataFmt'         : 1,        # 0=HW Streaming Disabled, 1=ADC, 4=CP_ADC_CQ
                              'enableSW'        : 0},       # 0=Disable User Data, 1=Enable User Data (HSI header must be enabled)

                        'calibMonCfg':
                            {'calibMonTimeUnit' : 0,        # 0  periodic Calibration and monitoring is DISABLED, else valid value (>0) to enable.
                             'calibPeriodicity' : 0}        # Calibration periodicity calibPeriodicity = 0: to disable periodic calibration, value (>0) to set the calibration period.
                        }
        else:
            # if path given, instead set mmwaveDict to parsed json file
            self.parse_mmWaveConfigJSON(mmwaveConfigPath)

    # Primary Functions
    # %---------------------------------------------------------------------------------------------------------
    def parse_mmWaveConfigJSON(self,filePath):
        """Loads an already written and correctly formatted json file into config object.
        
        File path given during initialization is used to load file. Function checks if file is correctly formatted 
        and states where errors in formatting occured. Function is very rigid on Json formatting and dictionary stored in
        json needs to follow default dictionary format given in class constructor exactly. 
        :params string filePath: filePath to json file to loaded into class.
        """
        with open(filePath) as json_file:
            loadedDict = json.load(json_file) # load config file from json format
            # check if keys are correct
            error = False
            if list(loadedDict.keys())==list(self.commandList.keys()): # check if correct commands present in file
                for dictKey in loadedDict:
                    if len(loadedDict[dictKey])!=self.commandList[dictKey]:
                        error=True
            else:
                error=True

            if error:
                self.mmwaveParamDict = None
            else:
                self.mmwaveParamDict = loadedDict

    # %---------------------------------------------------------------------------------------------------------
    def parse_runtimeConfigJSON(self,filePath):
        """Loads an already written and correctly formatted json file into config object.
        
        File path given during initialization is used to load file. Function checks if file is correctly formatted 
        and states where errors in formatting occured. Function is very rigid on Json formatting and dictionary stored in
        json needs to follow default dictionary format given in class constructor exactly. 
        :params string filePath: filePath to json file to loaded into class.
        """
        with open(filePath) as json_file:
            loadedDict = json.load(json_file)
            if self.runtimeList == list(loadedDict.keys()):
                self.runtimeParamDict = loadedDict
            else:
                self.runtimeParamDict = None

    # %---------------------------------------------------------------------------------------------------------
    def get_mmWaveCommandString(self, cmd):
        """Takes in a command such as "channelCfg" which corresponds to a key in mmwaveParamDict and 
        returns the complete command as string will all setting such as "channelCfg 15 7 0\\n".
        
        Creates a list out the available keys in mmwaveParamDict and checks if cmd is in that list.
        If it is the command is valid and it extracts the settings dictionary associated with that command from
        mmwWaveParamDict. Function then removes any suffixes not supproted by the mmWaveDevice such as _0, _1 etc. 
        that need to be used in the json file to separate different keys that represent the same command. 
        Then adds each setting to command string to be returned and adds the \\n terminator to end of string. 
        :params string cmd: config command name
        :returns string cmdStr: full command with command name and all settings and terminator
        :rtype: string  
        """
        if cmd in list(self.mmwaveParamDict.keys()):
            settingsDict = self.mmwaveParamDict[cmd]
            cmdStr = cmd.split("_", 1)[0] # remove repeated command suffix
            for setting in settingsDict:
                cmdStr = cmdStr + " " + str(settingsDict[setting]) # append each setting with a space to command string
            cmdStr = cmdStr + "\n"
            return cmdStr
        else:
            return ""
    