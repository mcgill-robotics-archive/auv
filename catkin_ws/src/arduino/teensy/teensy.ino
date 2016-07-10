#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <auv_msgs/MotorCommands.h>
#include <auv_msgs/SolenoidCommands.h>

#include "defs.h"

ros::NodeHandle nh;

Servo myservo[8];

unsigned long externalTempSchedule = 0;
unsigned long powerMonitorSchedule = 0;
unsigned long timeLastMotorCommand = 0;
unsigned long lastSolenoidCommand = 0;
unsigned long MotorStatusSchedule = 0;
unsigned long MissionSchedule = 0;

int lastMotorCommands[8] = {0,0,0,0,0,0,0,0};

std_msgs::Float32 external_temperature_m;
std_msgs::Float32 computerVoltage_m;
std_msgs::Float32 computerCurrent_m;
std_msgs::Float32 motorVoltage_m;
std_msgs::Float32 motorCurrent_m;
std_msgs::Bool mission_m;

ros::Publisher computerVoltagePub("~computer_voltage", &computerVoltage_m);
ros::Publisher ComputerCurrentPub("~computer_current", &computerCurrent_m);
ros::Publisher motorVoltagePub("~motor_voltage", &motorVoltage_m);
ros::Publisher motorCurrentPub("~motor_current", &motorCurrent_m);

ros::Publisher missionPub("/mission",&mission_m);

int boundCheck(int motorCommandValue){
  if(motorCommandValue> 500 || motorCommandValue< -500){
    char msg[80];
    String("Motor command out of bound: " + String(motorCommandValue) +" !").toCharArray(msg,80);
    nh.logerror(msg);
    return 0;
  }
  return motorCommandValue;
}

void writeMotorT100 (uint8_t motorNumber, int motorCommandValue)
{
  int difference = motorCommandValue-lastMotorCommands[motorNumber];

  if(abs(difference) < THRESHOLD_MOTOR)
    lastMotorCommands[motorNumber] = boundCheck(motorCommandValue);
  else if (difference > 0)
    lastMotorCommands[motorNumber] = boundCheck(lastMotorCommands[motorNumber] + THRESHOLD_MOTOR);
  else
    lastMotorCommands[motorNumber] = boundCheck(lastMotorCommands[motorNumber] - THRESHOLD_MOTOR);
  myservo[motorNumber].writeMicroseconds(MOTOR_T100_RST_VALUE + lastMotorCommands[motorNumber]);
}

void motorCb( const auv_msgs::MotorCommands& msg){
  if(mission_m.data){
    timeLastMotorCommand = millis();
    writeMotorT100(MOTOR_PIN_PORT_SURGE, msg.stern_sway);
    writeMotorT100(MOTOR_PIN_STARBOARD_SURGE, msg.bow_sway);
    writeMotorT100(MOTOR_PIN_PORT_BOW_HEAVE, msg.starboard_bow_heave);
    writeMotorT100(MOTOR_PIN_STARBOARD_BOW_HEAVE, msg.port_bow_heave);
    writeMotorT100(MOTOR_PIN_PORT_STERN_HEAVE, msg.starboard_stern_heave);
    writeMotorT100(MOTOR_PIN_STARBOARD_STERN_HEAVE, msg.port_stern_heave);
    writeMotorT100(MOTOR_PIN_STARBOARD_SWAY, msg.port_surge);
    writeMotorT100(MOTOR_PIN_PORT_SWAY, msg.starboard_surge);
  } else {
    nh.logwarn("Motor commands received while mission off!! Commands IGNORED!!");
  }
}

void resetMotor(){
  for(int i = 0; i < 8; i++){
    lastMotorCommands[i] = 0;
    myservo[i].writeMicroseconds(MOTOR_T100_RST_VALUE);
  }
  nh.loginfo("Motors got reset!");
}

void resetSolenoid(){
  digitalWrite(SOLENOID_PIN_PORT_DROPPER, LOW);
  digitalWrite(SOLENOID_PIN_STARBOARD_DROPPER, LOW);
  digitalWrite(SOLENOID_PIN_PORT_GRABBER, LOW);
  digitalWrite(SOLENOID_PIN_STARBOARD_GRABBER, LOW);
  digitalWrite(SOLENOID_PIN_PORT_TORPEDO, LOW);
  digitalWrite(SOLENOID_PIN_STARBOARD_TORPEDO, LOW);
  digitalWrite(SOLENOID_PIN_EXTRA,LOW);
}

void solenoidCb( const auv_msgs::SolenoidCommands& msg){
  if(mission_m.data){
    lastSolenoidCommand= millis();
    digitalWrite(SOLENOID_PIN_PORT_DROPPER,msg.port_dropper);
    digitalWrite(SOLENOID_PIN_STARBOARD_DROPPER,msg.starboard_dropper);
    digitalWrite(SOLENOID_PIN_PORT_GRABBER,msg.port_grabber);
    digitalWrite(SOLENOID_PIN_STARBOARD_GRABBER,msg.starboard_grabber);
    digitalWrite(SOLENOID_PIN_PORT_TORPEDO,msg.port_torpedo);
    digitalWrite(SOLENOID_PIN_STARBOARD_TORPEDO,msg.starboard_torpedo);
    digitalWrite(SOLENOID_PIN_EXTRA,msg.extra);
  } else {
    nh.logwarn("Solenoid commands received while mission off!! Commands IGNORED!!");
  }
}

void inline toggleLed(){
  digitalWrite(LED_PIN,!digitalRead(LED_PIN));
}

ros::Subscriber<auv_msgs::SolenoidCommands> solenoidSub("~solenoid", &solenoidCb );
ros::Subscriber<auv_msgs::MotorCommands> motorSub("~motor", &motorCb );

void motorInit(){

  //Setup for T100, normal servo control
  myservo[MOTOR_PIN_PORT_SURGE].attach(MOTOR_PIN_PORT_SURGE);
  myservo[MOTOR_PIN_STARBOARD_SURGE].attach(MOTOR_PIN_STARBOARD_SURGE);
  myservo[MOTOR_PIN_PORT_BOW_HEAVE].attach(MOTOR_PIN_PORT_BOW_HEAVE);
  myservo[MOTOR_PIN_STARBOARD_BOW_HEAVE].attach(MOTOR_PIN_STARBOARD_BOW_HEAVE);
  myservo[MOTOR_PIN_PORT_STERN_HEAVE].attach(MOTOR_PIN_PORT_STERN_HEAVE);
  myservo[MOTOR_PIN_STARBOARD_STERN_HEAVE].attach(MOTOR_PIN_STARBOARD_STERN_HEAVE);
  myservo[MOTOR_PIN_STARBOARD_SWAY].attach(MOTOR_PIN_STARBOARD_SWAY);
  myservo[MOTOR_PIN_PORT_SWAY].attach(MOTOR_PIN_PORT_SWAY);

  resetMotor();
}

void solenoidInit(){

  pinMode(SOLENOID_PIN_PORT_DROPPER, OUTPUT);
  pinMode(SOLENOID_PIN_STARBOARD_DROPPER, OUTPUT);
  pinMode(SOLENOID_PIN_PORT_GRABBER, OUTPUT);
  pinMode(SOLENOID_PIN_STARBOARD_GRABBER, OUTPUT);
  pinMode(SOLENOID_PIN_PORT_TORPEDO, OUTPUT);
  pinMode(SOLENOID_PIN_STARBOARD_TORPEDO, OUTPUT);
  pinMode(SOLENOID_PIN_EXTRA, OUTPUT);
  resetSolenoid();

}

void gpioInit(){

  //Analog Setup
  pinMode(COMPUTER_VOLTAGE_PIN,INPUT);
  pinMode(COMPUTER_CURRENT_PIN,INPUT);
  pinMode(MOTOR_VOLTAGE_PIN,INPUT);
  pinMode(MOTOR_CURRENT_PIN,INPUT);
  pinMode(MISSION_PIN,INPUT);

  //Onboard LED setup
  pinMode(LED_PIN,OUTPUT);

}


void rosInit(){
  //ros node initialization
  nh.initNode();
  //ros publisher initialization
  nh.advertise(computerVoltagePub);     //battery level
  nh.advertise(ComputerCurrentPub);
  nh.advertise(motorVoltagePub);
  nh.advertise(motorCurrentPub);
  nh.advertise(missionPub);

  //ros subscribe initialization
  nh.subscribe(motorSub);
  nh.subscribe(solenoidSub);
}

void setup(){

  motorInit();
  solenoidInit();
  gpioInit();

  rosInit();
}

void loop(){
  unsigned long currentTime = millis();
  mission_m.data = digitalRead(MISSION_PIN);

  if(powerMonitorSchedule < currentTime){
    computerVoltage_m.data = analogRead(COMPUTER_VOLTAGE_PIN) * kCOM_VOLT_SLOPE + kCOM_VOLT_OFFSET;
    computerCurrent_m.data = analogRead(COMPUTER_CURRENT_PIN) * kCOM_CURR_SLOPE + kCOM_CURR_OFFSET;
    motorVoltage_m.data = analogRead(MOTOR_VOLTAGE_PIN) * kMOT_VOLT_SLOPE + kMOT_VOLT_OFFSET;
    motorCurrent_m.data = analogRead(MOTOR_CURRENT_PIN) * kMOT_CURR_SLOPE + kMOT_CURR_OFFSET;
    computerVoltagePub.publish(&computerVoltage_m);
    ComputerCurrentPub.publish(&computerCurrent_m);
    motorVoltagePub.publish(&motorVoltage_m);
    motorCurrentPub.publish(&motorCurrent_m);
    powerMonitorSchedule += POWER_MONITOR_INTERVAL;
    toggleLed();
  }

  //Seabotix Motor status
  if(MissionSchedule < currentTime){
    missionPub.publish(&mission_m);
    MissionSchedule += MISSION_INTERVAL;
    toggleLed();
  }

  //Motor
  if(lastSolenoidCommand + MOTOR_TIMEOUT < currentTime){
    //nh.logerror("Solenoid Command timeout!");
    resetSolenoid();
    lastSolenoidCommand = currentTime;
    toggleLed();
  }

  if(timeLastMotorCommand + MOTOR_TIMEOUT < currentTime){
    if(mission_m.data){
      nh.logwarn("Motor commands timeout!");
      timeLastMotorCommand = currentTime;
      toggleLed();
    } else {
      timeLastMotorCommand = currentTime + MOTOR_TIMEOUT;
      toggleLed();
    }
    resetMotor();
  }
  nh.spinOnce();
}
