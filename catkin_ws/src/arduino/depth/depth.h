#ifndef DEPTH_H
#define DEPTH_H

#ifdef ARDUINO_AVR_FEATHER32U4
#define USE_USBCON
#endif

#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>

#include "ms5803_i2c.h"
#include "ros.h"
#include "std_msgs/Float32.h"

#define MS5803_I2C_ADDR 0x77
#define PIN_LED 13
#define MCUSR_WDRF 0b00001000
#define TEMPERATURE_REPORT_INTERVAL 1000
#define PRESSURE_REPORT_INTERVAL 100
#define PRESSURE_RECONNECT_INTERVAL 5000

ros::NodeHandle nh;

std_msgs::Float32 pressure_m;
std_msgs::Float32 temperature_m;

ros::Publisher pressure_pub("~pressure", &pressure_m);
ros::Publisher temperature_pub("~temperature", &temperature_m);

MS5803 pressure_sensor(MS5803_I2C_ADDR);

bool pressure_sensor_connected = false;
unsigned long pressure_schedule = 0;
unsigned long temperature_schedule = 0;

#endif  // DEPTH_H
