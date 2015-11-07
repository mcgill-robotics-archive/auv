//#define USE_USB_SERIAL

#include "ros.h"
#include <auv_msgs/RadioCommands.h>

ros::NodeHandle nh;

auv_msgs::RadioCommands radio_msg;
ros::Publisher radio_pub("radio", &radio_msg);


#define PIN_CHANNEL1          21
#define PIN_CHANNEL2          20
#define PIN_CHANNEL3          19
#define PIN_CHANNEL4          18
#define PIN_CHANNEL5          17
#define PIN_CHANNEL6          16
#define PIN_CHANNEL7          15
#define PIN_CHANNEL8          14

#define PIN_LED               13

#define MAX_BOUND             2100
#define MIN_BOUND             900
#define RADIO_RESET_VALUE     1500

#define TRANSMIT_ENABLE_PIN   11

elapsedMillis radio_publish_timer;     // a timer to schehule publishing data
elapsedMillis radio_timeout;           // a timer to detech singal lost

elapsedMicros pulse_timer [8];         // timer to measure pulse width
uint16_t pulseWidth[8];                // variable to store data


void pulseCalc(uint8_t index, uint8_t pin) {
  if( !(index == 4) && !(index == 3)){
    radio_timeout = 0;
  }
    
  
  if(digitalRead(pin) == HIGH) {
    pulse_timer[index] = 0;
  }
  else {
    pulseWidth[index] = pulse_timer[index];
  }
}


//pin0 ISR
void calc0() {
  pulseCalc(0, PIN_CHANNEL1);
}

// pin1 ISR
void calc1() {
  pulseCalc(1, PIN_CHANNEL2);
}

// pin2 ISR
void calc2() {
  pulseCalc(2, PIN_CHANNEL3);
}

// pin3 ISR
void calc3() {
  pulseCalc(3, PIN_CHANNEL4);
}

// pin4 ISR
void calc4() {
  pulseCalc(4, PIN_CHANNEL5);
}

// pin5 ISR
void calc5() {
  pulseCalc(5, PIN_CHANNEL6);
}

// pin6 ISR
void calc6() {
  pulseCalc(6, PIN_CHANNEL7);
}

// pin7 ISR
void calc7() {
  pulseCalc(7, PIN_CHANNEL8);
}

//---------------------------------------------------------------


void setup() {
  
    // Attach ISRs to pins

    pinMode(PIN_CHANNEL1, INPUT_PULLUP);
    attachInterrupt(PIN_CHANNEL1, calc0, CHANGE);

    pinMode(PIN_CHANNEL2, INPUT_PULLUP);
    attachInterrupt(PIN_CHANNEL2, calc1, CHANGE);

    pinMode(PIN_CHANNEL3, INPUT_PULLUP);
    attachInterrupt(PIN_CHANNEL3, calc2, CHANGE);

    pinMode(PIN_CHANNEL4, INPUT_PULLUP);
    attachInterrupt(PIN_CHANNEL4, calc3, CHANGE);

    pinMode(PIN_CHANNEL5, INPUT_PULLUP);
    attachInterrupt(PIN_CHANNEL5, calc4, CHANGE);

    pinMode(PIN_CHANNEL6, INPUT_PULLUP);
    attachInterrupt(PIN_CHANNEL6, calc5, CHANGE);

    pinMode(PIN_CHANNEL7, INPUT_PULLUP);
    attachInterrupt(PIN_CHANNEL7, calc6, CHANGE);

    pinMode(PIN_CHANNEL8, INPUT_PULLUP);
    attachInterrupt(PIN_CHANNEL8, calc7, CHANGE);

    //----------------------------------------------------------------- 
  
    //ROS initialization
    nh.initNode();
    nh.advertise(radio_pub);
    
#ifndef USE_USB_SERIAL
    Serial2.transmitterEnable(TRANSMIT_ENABLE_PIN);
#endif

    pinMode(PIN_LED,OUTPUT);

}

uint16_t boundcheck(bool timeout, uint16_t value){
    if (timeout){
        return 1500;
    } else if(value >  MAX_BOUND){
        return MAX_BOUND;
    } else if(value < MIN_BOUND){
        return MIN_BOUND;
    } else {
        return value;    
    }
    
}

void loop() {
    if(radio_publish_timer > 50){
      bool timed_out = radio_timeout > 20;
      radio_msg.channel1 = boundcheck(timed_out, pulseWidth[0]);
      radio_msg.channel2 = boundcheck(timed_out, pulseWidth[1]);
      radio_msg.channel3 = boundcheck(timed_out, pulseWidth[2]);
      radio_msg.channel4 = boundcheck(timed_out, pulseWidth[3]);
      radio_msg.channel5 = boundcheck(timed_out, pulseWidth[4]);
      radio_msg.channel6 = boundcheck(timed_out, pulseWidth[5]);
      radio_msg.channel7 = boundcheck(timed_out, pulseWidth[6]);
      radio_msg.channel8 = boundcheck(timed_out, pulseWidth[7]);
      radio_pub.publish( &radio_msg );
      radio_publish_timer = 0;
      digitalWrite(PIN_LED,!digitalRead(PIN_LED));
    }  
    
    nh.spinOnce();
  
}
