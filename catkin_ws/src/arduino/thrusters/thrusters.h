#ifndef MAIN_H
#define MAIN_H
#include <Arduino.h>
#include <Servo.h>
#include <auv_msgs/ThrusterCommands.h>
#include <avr/wdt.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "ros.h"

#define MCUSR_WDRF 0b00001000

ros::NodeHandle nh;

#define THRUSTER_COUNT 8
#define THRUSTER_RST_VALUE 1500
#define THRUSTER_MAX_INCREMENT 50
#define THRUSTER_TIMEOUT 500
#define TRRUSTER_RESET_INTERVAL 10000

unsigned long thruster_reset_schedule = 0;
const uint8_t thruster_pins[THRUSTER_COUNT] = {A2, A3, A0, A1, 5, 2, 3, 4};
int16_t last_thruster_commands[THRUSTER_COUNT] = {0};
Servo thrusters[THRUSTER_COUNT];

void thrustersCallback(const auv_msgs::ThrusterCommands& msg);
ros::Subscriber<auv_msgs::ThrusterCommands> thruster_sub("~thrusters",
                                                         &thrustersCallback);

#define MISSION_REPORT_INTERVAL 100
#define MISSION_PIN A5

unsigned long misssion_report_schedule = 0;
bool mission_enabled = false;
std_msgs::Bool mission_m;
ros::Publisher mission_pub("~mission", &mission_m);

#define VOLTAGE_REPORT_INTERVAL 500
#define VOLT_SLOPE 3.3 / 1023.0 / 0.1282;
#define VOLT_OFFSET 0.6;
#define VOLTAGE_PIN A4

unsigned long voltage_report_schedule = 0;

std_msgs::Float32 voltage_m;
ros::Publisher voltage_pub("~voltage", &voltage_m);

#define LED_PIN 13
void inline toggleLed() { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); }
#endif  // MAIN_H
