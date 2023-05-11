# importing required libraries
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtPrintSupport import *
import os
import sys
import threading

from m2s2_pydevclass.m2s2_device import M2S2Device

# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# ROS2 Interface imports 
from m2s2_interfaces.srv import SendDeviceControlCommand

# global variables
node = None
path = None
 
# Creating main window class
class MainWindow(QMainWindow):

    cmdDICT = {
        "SETUP":2,
        "START":1,
        "STOP":0
    }
 
    # constructor
    def __init__(self, *args, **kwargs):
        global path
        super(MainWindow, self).__init__(*args, **kwargs)
 
        # setting window geometry
        self.setGeometry(100, 100, 600, 400)
 
        # creating a layout
        self.mainwidget = QWidget()
        layout = QHBoxLayout(self.mainwidget)
 
        # creating a QPlainTextEdit object
        self.editor = QPlainTextEdit()
 
        # setting font to the editor
        fixedfont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        fixedfont.setPointSize(12)
        self.editor.setFont(fixedfont)
 
        # self.path holds the path of the currently open file.
        # If none, we haven't got a file open yet (or creating new).
        self.path = path
        if path:
            try:
                with open(path, 'rU') as f:
                    text = f.read()
            except Exception as e:
                self.dialog_critical(str(e))
            else:
                self.path = path                # update path value
                self.editor.setPlainText(text)  # update the text

        # Control Panel
        self.controlPanel = QWidget()
        
        # create setup button
        setupButton = QPushButton()
        setupButton.setText("Setup Radar")
        setupButton.clicked.connect(self.setup_radar)

        # create start button
        startButton = QPushButton()
        startButton.setText("Start Recording")
        startButton.clicked.connect(self.start_radar)

        # create start button
        stopButton = QPushButton()
        stopButton.setText("Stop Recording")
        stopButton.clicked.connect(self.stop_radar)

        controlLayout = QFormLayout()
        self.controlPanel.setLayout(controlLayout)

        controlLayout.addRow(setupButton)
        controlLayout.addRow(startButton)
        controlLayout.addRow(stopButton)
 
        # adding editor to the layout
        layout.addWidget(self.editor)
        layout.addWidget(self.controlPanel)
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
        self.setWindowTitle("Radar Control Window")
 
        # showing all the components
        self.show()

    def setup_radar(self):
        response = node.send_request(self.cmdDICT["SETUP"])
        print(response)

    def start_radar(self):
        response = node.send_request(self.cmdDICT["START"])
        print(response)

    def stop_radar(self):
        response = node.send_request(self.cmdDICT["STOP"])
        print(response)

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

class MMWaveClientNode(Node):
    # %---------------------------------------------------------------------------------------------------------
    # Class Parameters
    # %---------------------------------------------------------------------------------------------------------
    
    # M2S2 Device Specification 
    device = None

    def __init__(self):
        global path

        ## Start Node
        self.device = M2S2Device("RADAR")
        super().__init__("%s_client_node" % self.device.devNAME) 

        self.declare_parameter('dev_cfg_file_path', '/home/orin/m2s2_ws/src/radar_ROS2_driver/cfgDev.json')
        self.devConfigPath = self.get_parameter('dev_cfg_file_path').get_parameter_value().string_value
        path = self.devConfigPath

        self.cli = self.create_client(SendDeviceControlCommand, self.device.devSERV)
        #print("dca1000/" + self.device.devSERV)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SendDeviceControlCommand.Request()

    def send_request(self, cmd):
        self.get_logger().info('Sending Request...')
        self.req.command = cmd
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    

def main(args=None):
    global node
    global path
    rclpy.init()
    node = MMWaveClientNode()
    path = node.devConfigPath
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    ros2ExecThread = threading.Thread(target=executor.spin,daemon=True)
    ros2ExecThread.start()

    app = QApplication(sys.argv)
    ard = MainWindow()
    ard.show()
    app.exec()

    print("Shutting Down")
    rclpy.shutdown()
    ros2ExecThread.join()



