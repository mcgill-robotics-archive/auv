#include "depth.h"

void resetPressureSensor() {
  Wire.beginTransmission(MS5803_I2C_ADDR);
  switch (Wire.endTransmission()) {
    case 0:
      pressure_sensor_connected = true;
      pressure_sensor.reset();
      pressure_sensor.begin();
      break;
    case 2:
      nh.logfatal("Pressure sensor address not attack!");
      break;
    default:
      nh.logfatal("Pressure sensor unknown error!");
  }
}

void reconnectPressureSensor() {
  resetPressureSensor();
  if (pressure_sensor_connected) {
    nh.logwarn("Pressure sensor reconnected.");
  } else {
    nh.logfatal("Pressure sensor disconnected!");
  }
}

void sensorInit() { resetPressureSensor(); }

void rosInit() {
  nh.initNode();
  nh.advertise(pressure_pub);
  nh.advertise(temperature_pub);
}

void setup() {
  sensorInit();
  rosInit();
  pinMode(PIN_LED, OUTPUT);
  if (MCUSR & MCUSR_WDRF) {
    MCUSR ^= MCUSR_WDRF;
    nh.logerror("Watchdog reset occurred!");
    nh.spinOnce();
  }
  wdt_enable(WDTO_2S);
}

void loop() {
  unsigned long time_now = millis();
  wdt_reset();
  // Depth Sensing
  if (pressure_schedule < time_now) {
    if (pressure_sensor_connected) {
      // Get Readings
      pressure_sensor.getMeasurements();

      // check status, reconnect if needed
      if (pressure_sensor.getStatus()) {
        pressure_sensor_connected = false;
        reconnectPressureSensor();

      } else {
        pressure_m.data = pressure_sensor.getPressure();
        pressure_pub.publish(&pressure_m);
        pressure_schedule += PRESSURE_REPORT_INTERVAL;
      }
    } else {
      pressure_schedule += PRESSURE_RECONNECT_INTERVAL;
      reconnectPressureSensor();
    }
  }

  // external Temperature
  if (temperature_schedule < time_now) {
    if (pressure_sensor_connected) {
      temperature_m.data = pressure_sensor.getTemperature();
      temperature_pub.publish(&temperature_m);
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }
    temperature_schedule += TEMPERATURE_REPORT_INTERVAL;
  }
  nh.spinOnce();
}
