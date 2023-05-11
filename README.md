![logo](docs/resources/ARU_logo_rectangle.png)
# TI Radar ROS2 Driver

## Description 

This repo aims provides a python [ROS2](https://docs.ros.org/en/foxy/index.html) driver for the Texas Instruments DCA1000 Raw ADC capture board and compatible antenna boards. Support for the Texas Instruments mmWave Cascaded system will be added in future. The ROS2 driver was initially based off of the repository by  [Moodoki](https://github.com/moodoki/iwr_raw_rosnode). 

<hr/> 

## Prerequisites

### ROS2

- Tested on [ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html)

- Using [eCAL RWM](https://github.com/eclipse-ecal/rmw_ecal) as an alternative to ROS2 DDS implementations showed significant perfomance improvements

### Python

- Tested with Python3.8 on Ubuntu 20.04 LTS

### Add user to dialout

- For serial commands to work, the current user needs to be part of the dialout group. 

```bash
sudo adduser $USER dialout
```
<hr/>

## Running the ros package

clone this repo to your ros2 workspace source directory:
```bash
git clone https://github.com/NicNedwob/RadarRepo2.git
```
Modify radar parameters as required in the [device config](radar_dca1000_py_pkg/cfgDev.json) file and the runtime configs as needed in the [runtime config](radar_dca1000_py_pkg/cfgRun.json) file, and run the node using the provided launch file or a custom. 
