#include <ros.h>
#include <arduino_msgs/Solenoid.h>

#define SOLENOID_PIN_1 4
#define SOLENOID_PIN_2 5
#define SOLENOID_PIN_3 6
#define SOLENOID_PIN_4 7
#define SOLENOID_PIN_5 8
#define SOLENOID_PIN_6 9

unsigned long lastSolenoidCommand = 0;

void solenoid_Cb( const arduino_msgs::Solenoid& msg){
  
  digitalWrite(SOLENOID_PIN_1,msg.solenoid1);
  digitalWrite(SOLENOID_PIN_2,msg.solenoid2);
  digitalWrite(SOLENOID_PIN_3,msg.solenoid3);
  digitalWrite(SOLENOID_PIN_4,msg.solenoid4);
  digitalWrite(SOLENOID_PIN_5,msg.solenoid5);
  digitalWrite(SOLENOID_PIN_6,msg.solenoid6);
  
  lastSolenoidCommand= millis();
}


ros::NodeHandle nh;
ros::Subscriber<arduino_msgs::Solenoid> solenoid_Sub("/solenoid", &solenoid_Cb );





void resetSolenoid(){
  
  digitalWrite(SOLENOID_PIN_1,LOW);
  digitalWrite(SOLENOID_PIN_2,LOW);
  digitalWrite(SOLENOID_PIN_3,LOW);
  digitalWrite(SOLENOID_PIN_4,LOW);
  digitalWrite(SOLENOID_PIN_5,LOW);
  digitalWrite(SOLENOID_PIN_6,LOW);
  
  lastSolenoidCommand= millis();

}

void setup(){

  pinMode(SOLENOID_PIN_1,OUTPUT);
  pinMode(SOLENOID_PIN_2,OUTPUT);
  pinMode(SOLENOID_PIN_3,OUTPUT);
  pinMode(SOLENOID_PIN_4,OUTPUT);
  pinMode(SOLENOID_PIN_5,OUTPUT);
  pinMode(SOLENOID_PIN_6,OUTPUT);  
  
  resetSolenoid();
  
  nh.initNode();
  nh.subscribe(solenoid_Sub);
  
  resetSolenoid();

}

void loop(){
  unsigned long currentTime = millis();
  
  if(currentTime - lastSolenoidCommand > 200){
    resetSolenoid();
  }  
  nh.spinOnce();  

}

