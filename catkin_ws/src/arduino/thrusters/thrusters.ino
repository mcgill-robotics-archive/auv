#include "thrusters.h"

int boundCheck(int input) {
  if (abs(input) > 500) {
    nh.logerror("Thruster command out of bound!");
    return max(-300, min(300, input));
  }
  return input;
}

void thrustersCallback(const auv_msgs::ThrusterCommands& msg) {
  if (mission_enabled) {
    thruster_reset_schedule = millis() + THRUSTER_TIMEOUT;

    for (uint8_t i = 0; i < THRUSTER_COUNT; i++) {
      int16_t value = boundCheck(msg.thruster_commands[i]);
      int16_t last_thruster_command = last_thruster_commands[i];

      value = max(last_thruster_command - THRUSTER_MAX_INCREMENT, value);
      value = min(last_thruster_command + THRUSTER_MAX_INCREMENT, value);

      thrusters[i].writeMicroseconds(THRUSTER_RST_VALUE + value);
      last_thruster_commands[i] = value;
    }
  } else {
    nh.logwarn("Mission off, thrusterCommands IGNORED!!");
  }
}

void resetThrusters() {
  for (uint8_t i = 0; i < THRUSTER_COUNT; i++) {
    last_thruster_commands[i] = 0;
    thrusters[i].writeMicroseconds(THRUSTER_RST_VALUE);
  }
  nh.loginfo("Thrusters reset!!");
}

void thrustersInit() {
  for (uint8_t i = 0; i < THRUSTER_COUNT; i++) {
    thrusters[i].attach(thruster_pins[i]);
  }
  resetThrusters();
}

void gpioInit() {
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(MISSION_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
}

void rosInit() {
  nh.initNode();

  //nh.advertise(voltage_pub);
  nh.advertise(mission_pub);
  nh.subscribe(thruster_sub);
}

void setup() {
  thrustersInit();
  gpioInit();
  rosInit();
  if (MCUSR & MCUSR_WDRF) {
    MCUSR ^= MCUSR_WDRF;
    nh.logerror("Watchdog reset occurred!");
    nh.spinOnce();
  }
  wdt_enable(WDTO_1S);
}

void voltageReportTask(unsigned long time_now) {
  if (voltage_report_schedule < time_now) {
    voltage_m.data = analogRead(VOLTAGE_PIN) * VOLT_SLOPE + VOLT_OFFSET;
    voltage_pub.publish(&voltage_m);
    voltage_report_schedule += VOLTAGE_REPORT_INTERVAL;
    toggleLed();
  }
}

void thursterResetTask(unsigned long time_now) {
  if (thruster_reset_schedule < time_now) {
    if (mission_enabled) {
      nh.logwarn("Motor commands timeout!");
    }
    resetThrusters();
    thruster_reset_schedule = time_now + TRRUSTER_RESET_INTERVAL;
    toggleLed();
  }
}

void missionReportTask(unsigned long time_now) {
  if (misssion_report_schedule < time_now) {
    mission_m.data = mission_enabled;
    mission_pub.publish(&mission_m);
    misssion_report_schedule += MISSION_REPORT_INTERVAL;
    toggleLed();
  }
}
void loop() {
  wdt_reset();
  unsigned long time_now = millis();
  mission_enabled = digitalRead(MISSION_PIN);

  thursterResetTask(time_now);
  missionReportTask(time_now);
  //voltageReportTask(time_now);

  nh.spinOnce();
}
