#include <iostream>
#include <fstream>
#include <string.h>
#include <jsoncpp/json/json.h>

using namespace std;
int maskToNum(long n) {
    long i;
    int num = n;
    int sum = 0;

    for (i = 0; i <4 ; i++)
    {
      sum += num%2;
      num = num/2;
    }
    return sum;
 }

int main(){

    cout << "Json Read Test" << endl;

    // read file
    ifstream file("/home/nedwob/ros2_ws/src/MyM2S2Code/radar_ROS2_driver/cfgDev.json");
    // json reader
    Json::Reader reader;

        string devCfgPath ="/home/nedwob/ros2_ws/src/MyM2S2Code/radar_ROS2_driver/cfgDev.json";
        ifstream dev(devCfgPath);
        Json::Value devCfgData;
        reader.parse(dev, devCfgData);

        int numChirps = devCfgData["frameCfg"]["numChirps"].asInt();
        int numSamples = devCfgData["profileCfg"]["numAdcSamples"].asInt();
        int bytesPerSample = 2; // adc either returns 12,14 or 16bit data which all have to be stored over 2 bytes 
        int realOrComplex = (devCfgData["adcbufCfg"]["adcOutputFmt"].asInt()+2) % 3; //if 0 it is complex so 2%3 = 2, if 1 it is real 3%3=1
        int numReceivers = maskToNum(devCfgData["channelCfg"]["rxChannelEn"].asInt()); // convert number to bit mask string with bin() and remove 0b from start of string
        cout << numChirps << endl;
        cout << numSamples<< endl;
        cout << bytesPerSample<< endl;
        cout << realOrComplex<< endl;
        cout << numReceivers << endl;

        int frameSize = numChirps*numSamples*bytesPerSample*realOrComplex*numReceivers; // calculate required buffer Size
        cout << "Frame Size: " << frameSize << endl;

        cout << endl;
        cout << "Data Vector Instantiate Test" << endl;
        int end = 6;
        int array[] = {0,1,2,3,4,5,6};
        std::vector<int> data_vector(array, array + end);
        for (int i = 0 ; i < end; i++)
        {
          cout << data_vector[i] << endl;
        }
        cout << endl;

        cout << "Packet Num Extraction Test" << endl;
        unsigned char incoming_udp_packet[] = {0x05,0x01,0x00,0x00,0x42,0x58,0x42,0x58,0x42,0x58,0x41,0x42};
        int packetNum = 0;
        for (int i = 0; i < 4 ;i++){
            packetNum = packetNum | (incoming_udp_packet[i]) << i*8;
        } 

        cout << packetNum << endl;
        cout << endl;

        cout << "Packet Data Extraction Test" << endl;
        unsigned char data[8] = {0x00};
        memcpy(data,incoming_udp_packet+4,8);
        for(int i=0;i<8;i++){
          cout << data[i];  
        } cout << endl;

}
