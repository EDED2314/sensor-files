#include <TRSim_Up.h>
TRSim_Up::Simulator TRsim = TRSim_Up::Simulator(); 

int num_packets = 0;
uint16_t prev_events = 0;

void setup() {
  
  Serial.begin(19200);
 // TRSim_Up::Simulator yourmom = TRSim_Up::Simulator(); 
  

}

void loop() {
  TRsim.init(2,3);
  TRsim.update();
  bool x = TRsim.isStreaming();
  if(x){
    if(TRsim.isNewData()){
      num_packets++;
      unsigned char* data = TRsim.getData();
      uint16_t curr_events = TRsim.getEvents();
      if(curr_events != prev_events){
        prev_events = curr_events;

                    
      }
      
      
    }
  }
  

}
