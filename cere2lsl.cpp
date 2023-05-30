/*
This program builds the cereBridge to LSL connector for the SensOhr 
platform. Build on the Sensohr w/ 
g++ -o cere2lsl cere2lsl.cpp -lwiringPi -llsl
start with ./cere2lsl
LSL and wiringPi libraries need to be installed.
*/


#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <limits.h>
#include <getopt.h>
#include <string>
#include "lsl_cpp.h"
#include <array>
#include <tuple>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <atomic>
constexpr int STAT_L=0xC00000;
constexpr int STAT_R=0xC0000F;


std::atomic<bool> exit_request(false);

void my_handler(int){
  exit_request=true;
}


// convert 3 byte 2's compliment to 4 byte 2's compliment
int transform(int value) {
  return (int)(value << 8) >> 8;
}

float toVolts(int int24) {
  float res=transform(int24)*0.536e-6;
  return res;
}

class commandLineOptions
{
public:
  commandLineOptions(int argc, char** argv){
    int c;
    while (1)
      {
        static struct option long_options[] =
          {
            /* These options don’t set a flag.
               We distinguish them by their indices. */
            {"device", required_argument, 0, 'd'},
            {"chunksize",  required_argument, 0, 'c'},
            {"sourceid",  required_argument, 0, 's'},
            {"streamname",    required_argument, 0, 'n'},
            {0, 0, 0, 0}
          };
        /* getopt_long stores the option index here. */
        int option_index = 0;

        c = getopt_long (argc, argv, "c:s:n:",
                         long_options, &option_index);

        /* Detect the end of the options. */
        if (c == -1)
          break;

        switch (c)
          {
          case 'd':
            deviceName=std::string(optarg);
            break;

          case 'c':
            chunkSize=std::stoi(optarg);
            break;

          case 's':
            sourceId=std::string(optarg);
            break;

          case 'n':
            streamName=std::string(optarg);
            break;

          case '?':
            /* getopt_long already printed an error message. */
            break;

          default:
            throw std::runtime_error("91");
          }
      }
    if(sourceId.empty()){
      sourceId.reserve(32);
      gethostname(&sourceId[0],32);
    }
  }

  std::string deviceName="/dev/serial1";
  std::string streamName="CereBridge";
  std::string sourceId;
  int chunkSize=0;
};

class serialConnection{
 public:
  serialConnection(const std::string& deviceName) {
    if ((serial_port = serialOpen (deviceName.c_str(), 460800)) < 0)	/* open serial port */
      {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        throw std::runtime_error("111");
      }

    if (wiringPiSetup () == -1)					/* initializes wiringPi setup */
      {
        fprintf (stderr, "Unable to start wiringPi: %s\n", strerror (errno)) ;
        throw std::runtime_error("117");
      }
    channels.resize(16);
    serialFlush(serial_port);
    wait_for_stat_l(serial_port);
  }

  virtual ~serialConnection(){
    serialClose(serial_port);
  }


  std::vector<float> get_samples(){
    fill_channels();
    return channels;
  }

 private:

  std::vector<float> channels;
  int serial_port;

  void fill_channels(){
    int frame=0;
    // We should currently be right after STAT_L
    // Get ADC_L_Ch1..ADC_L_Ch8
    for(int i=0;i<8;++i){
      frame=get_ch(serial_port);
      channels[i]=toVolts(frame);
    }
    // Sanity check: Next frame should be STAT_R
    frame=get_ch(serial_port);
    if(frame!=STAT_R){
      printf("Error: Did expect STAT_R. Got: %x \n",frame);
      throw std::runtime_error("141");
    }
    // Get ADC_R_Ch1..ADC_R_Ch8
    for(int i=8;i<16;++i){
      frame=get_ch(serial_port);
      channels[i]=toVolts(frame);
    }
    // Sanity check: Next frame should be STAT_L
    frame=get_ch(serial_port);
    if(frame!=STAT_L){
      printf("Error: Did expect STAT_L. Got: %x \n",frame);
      throw std::runtime_error("152");
    }
  }

  int get_ch(int serial_port){
    int res=0x0;
    int inByte=0x0;
    for(int i=0; i<3; i++){			//  read 3 bytes from ADS1299
      inByte = serialGetchar(serial_port);
      if(inByte==-1 and exit_request==false){
        printf("Error reading from serial port");
        throw std::runtime_error("167");
      }
      res = (res<<8) | inByte;
    }
    return res;
  }

  int wait_for_stat_l(int serial_port){
    while (true) {
      char dat=serialGetchar(serial_port);
      while( (dat=serialGetchar(serial_port) )!= 0xC0)
        ;
      char dat2=serialGetchar(serial_port);
      char dat3=serialGetchar(serial_port);
      if(dat2==0x0 and dat3==0x0){
        return 0;
      }
    }
  }
};

int main (int argc, char** argv)
{

  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

  commandLineOptions options(argc,argv);
  lsl::stream_info info(options.streamName,"eeg",16,250,lsl::cf_float32,options.sourceId);
  lsl::stream_outlet stream_out(info,options.chunkSize);
  serialConnection con(options.deviceName);

  try{
  while(!exit_request){
    stream_out.push_sample(con.get_samples());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  }
  catch(std::exception& e){
    // Ignore exceptions during exit for now
    if(exit_request)
      ;
    else
      throw e;
  }
  return 0;
}
